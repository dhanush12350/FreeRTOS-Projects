#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------
 * Industrial Thresholds & System Parameters
 * ------------------------------------------------------------ */
#define OVERCURRENT_LIMIT_AMPS      15.0f
#define OVERVOLTAGE_LIMIT_VOLTS    260.0f
#define UNDERVOLTAGE_LIMIT_VOLTS   180.0f
#define OVERTEMP_LIMIT_CELSIUS      85.0f

#define ADC_TASK_PERIOD_MS          10U   /* 100Hz Sampling Rate */
#define TEMP_TASK_PERIOD_MS        200U   /* 5Hz Sampling Rate */
#define PROC_TASK_PERIOD_MS         50U   /* 20Hz Processing Rate */
#define CAN_TASK_PERIOD_MS         100U   /* 10Hz Broadcast Rate */
#define WATCHDOG_PERIOD_MS         500U   /* 500ms Watchdog Timeout */

#define HEARTBEAT_ADC_BIT   ( 1 << 0 )
#define HEARTBEAT_TEMP_BIT  ( 1 << 1 )
#define HEARTBEAT_PROC_BIT  ( 1 << 2 )
#define ALL_HEARTBEATS      ( HEARTBEAT_ADC_BIT | HEARTBEAT_TEMP_BIT | HEARTBEAT_PROC_BIT )

#define FAULT_NONE          0x00U
#define FAULT_OVERCURRENT   0x01U
#define FAULT_OVERVOLTAGE   0x02U
#define FAULT_UNDERVOLTAGE  0x04U
#define FAULT_OVERTEMP      0x08U
#define FAULT_WATCHDOG_TRIP 0x10U

/* ------------------------------------------------------------
 * Telemetry & Bus Frame Structures
 * ------------------------------------------------------------ */
typedef struct {
    float    current_a;
    float    voltage_v;
    float    temp_c;
    uint32_t fault_mask;
    uint32_t timestamp_ms;
} TelemetryData_t;

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} CANFrame_t;

/* ------------------------------------------------------------
 * FreeRTOS Handles & Shared State
 * ------------------------------------------------------------ */
static SemaphoreHandle_t  xSensorMutex                = NULL;
static SemaphoreHandle_t  xADC_EmergencyTripSemaphore = NULL;
static QueueHandle_t      xTelemetryQueue             = NULL;
static EventGroupHandle_t xTaskHeartbeatEventGroup    = NULL;

static TelemetryData_t g_xSharedTelemetry = {0.0f, 230.0f, 25.0f, FAULT_NONE, 0U};
static volatile bool    g_bSystemTripped  = false;

/* ------------------------------------------------------------
 * Hardware Abstraction Layer (HAL) Driver Mocks
 * ------------------------------------------------------------ */
static void HAL_Relay_Trip(void)
{
    g_bSystemTripped = true;
    /* Instantaneous Hardware Disconnect (e.g. GPIO_ResetBits(RELAY_GPIO_PORT, RELAY_PIN)) */
}

static void HAL_Alarm_Set(bool bState)
{
    (void)bState;
    /* Hardware Alarm Siren / Flash LED */
}

static float HAL_ADC_ReadCurrent(void)
{
    return 10.5f; /* Standard nominal 10.5A */
}

static float HAL_ADC_ReadVoltage(void)
{
    return 230.0f; /* Standard nominal 230V */
}

static float HAL_I2C_ReadTemperature(void)
{
    return 42.5f; /* Standard nominal 42.5°C */
}

static void HAL_CAN_TransmitFrame(const CANFrame_t *pxFrame)
{
    (void)pxFrame;
    /* CAN Peripheral Tx Mailbox Register Write */
}

/* ------------------------------------------------------------
 * Hardware Analog Watchdog Interrupt Service Routine (ISR)
 * Overcurrent hardware comparator triggers this instantly (<1us)
 * ------------------------------------------------------------ */
void ADC_AnalogWatchdog_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Hardware Relay Trip executed directly in ISR for sub-microsecond protection */
    HAL_Relay_Trip();
    HAL_Alarm_Set(true);

    if (xADC_EmergencyTripSemaphore != NULL) {
        xSemaphoreGiveFromISR(xADC_EmergencyTripSemaphore, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ------------------------------------------------------------
 * Task 1: High-Speed ADC Protection Task (Priority 5 - 100Hz)
 * Samples Current & Voltage every 10ms with instant threshold trip
 * ------------------------------------------------------------ */
static void vHighSpeedADCProtectionTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ADC_TASK_PERIOD_MS));

        float fCurrent = HAL_ADC_ReadCurrent();
        float fVoltage = HAL_ADC_ReadVoltage();
        uint32_t ulLocalFault = FAULT_NONE;

        /* Evaluate Electrical Protection Limits */
        if (fCurrent > OVERCURRENT_LIMIT_AMPS) {
            ulLocalFault |= FAULT_OVERCURRENT;
        }
        if (fVoltage > OVERVOLTAGE_LIMIT_VOLTS) {
            ulLocalFault |= FAULT_OVERVOLTAGE;
        } else if (fVoltage < UNDERVOLTAGE_LIMIT_VOLTS) {
            ulLocalFault |= FAULT_UNDERVOLTAGE;
        }

        /* If electrical trip limit exceeded, open relay immediately */
        if (ulLocalFault != FAULT_NONE) {
            HAL_Relay_Trip();
            HAL_Alarm_Set(true);
        }

        /* Update shared sensor metrics safely using Mutex */
        if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(2U)) == pdTRUE) {
            g_xSharedTelemetry.current_a   = fCurrent;
            g_xSharedTelemetry.voltage_v   = fVoltage;
            g_xSharedTelemetry.fault_mask |= ulLocalFault;
            g_xSharedTelemetry.timestamp_ms = xTaskGetTickCount() ;
            xSemaphoreGive(xSensorMutex);
        }

        /* Signal Heartbeat to Watchdog */
        if (xTaskHeartbeatEventGroup != NULL) {
            xEventGroupSetBits(xTaskHeartbeatEventGroup, HEARTBEAT_ADC_BIT);
        }
    }
}

/* ------------------------------------------------------------
 * Task 2: I2C Digital Temperature Monitoring Task (Priority 3 - 5Hz)
 * Reads thermal sensors every 200ms
 * ------------------------------------------------------------ */
static void vI2CTemperatureTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TEMP_TASK_PERIOD_MS));

        float fTemp = HAL_I2C_ReadTemperature();
        uint32_t ulLocalFault = FAULT_NONE;

        if (fTemp > OVERTEMP_LIMIT_CELSIUS) {
            ulLocalFault |= FAULT_OVERTEMP;
            HAL_Relay_Trip();
            HAL_Alarm_Set(true);
        }

        if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(5U)) == pdTRUE) {
            g_xSharedTelemetry.temp_c       = fTemp;
            g_xSharedTelemetry.fault_mask  |= ulLocalFault;
            xSemaphoreGive(xSensorMutex);
        }

        if (xTaskHeartbeatEventGroup != NULL) {
            xEventGroupSetBits(xTaskHeartbeatEventGroup, HEARTBEAT_TEMP_BIT);
        }
    }
}

/* ------------------------------------------------------------
 * Task 3: Processing & Protection Logic Task (Priority 4 - 20Hz)
 * Evaluates holistic protection envelope and queues CAN telemetry
 * ------------------------------------------------------------ */
static void vProcessingProtectionTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    TelemetryData_t xLocalTelemetry;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PROC_TASK_PERIOD_MS));

        if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(5U)) == pdTRUE) {
            xLocalTelemetry = g_xSharedTelemetry;
            xSemaphoreGive(xSensorMutex);
        }

        /* Push telemetry snapshot to industrial telemetry queue */
        if (xTelemetryQueue != NULL) {
            xQueueSend(xTelemetryQueue, &xLocalTelemetry, 0U);
        }

        if (xTaskHeartbeatEventGroup != NULL) {
            xEventGroupSetBits(xTaskHeartbeatEventGroup, HEARTBEAT_PROC_BIT);
        }
    }
}

/* ------------------------------------------------------------
 * Task 4: Industrial CAN Bus Telemetry Task (Priority 2 - 10Hz)
 * Formats & broadcasts CAN 2.0B telemetry frames to PLC/SCADA
 * ------------------------------------------------------------ */
static void vIndustrialCANTask(void *pvParameters)
{
    (void)pvParameters;
    TelemetryData_t xTelemetry;
    CANFrame_t      xFrame;

    while (1) {
        if (xQueueReceive(xTelemetryQueue, &xTelemetry, portMAX_DELAY) == pdTRUE) {
            /* Format CAN Telemetry Frame 0x18F00100 */
            xFrame.id     = 0x18F00100U;
            xFrame.dlc    = 8U;

            uint16_t curr_scaled = (uint16_t)(xTelemetry.current_a * 10.0f);
            uint16_t volt_scaled = (uint16_t)(xTelemetry.voltage_v * 10.0f);
            uint8_t  temp_offset = (uint8_t)(xTelemetry.temp_c + 40.0f);

            xFrame.data[0] = (uint8_t)(curr_scaled & 0xFF);
            xFrame.data[1] = (uint8_t)((curr_scaled >> 8) & 0xFF);
            xFrame.data[2] = (uint8_t)(volt_scaled & 0xFF);
            xFrame.data[3] = (uint8_t)((volt_scaled >> 8) & 0xFF);
            xFrame.data[4] = temp_offset;
            xFrame.data[5] = (uint8_t)(xTelemetry.fault_mask & 0xFF);
            xFrame.data[6] = g_bSystemTripped ? 0xFFU : 0x00U;
            xFrame.data[7] = 0xAAU; /* Heartbeat signature byte */

            HAL_CAN_TransmitFrame(&xFrame);
        }
    }
}

/* ------------------------------------------------------------
 * Task 5: Ultimate Safety Watchdog Task (Priority 6 - Highest Priority)
 * Verifies that all worker tasks report heartbeats within 500ms
 * ------------------------------------------------------------ */
static void vSafetyWatchdogTask(void *pvParameters)
{
    (void)pvParameters;

    while (1) {
        EventBits_t uxBits = xEventGroupWaitBits(
            xTaskHeartbeatEventGroup,
            ALL_HEARTBEATS,
            pdTRUE,        /* Clear bits on exit */
            pdTRUE,        /* Wait for ALL bits */
            pdMS_TO_TICKS(WATCHDOG_PERIOD_MS)
        );

        /* If any task failed to check-in within 500ms, trip system relay */
        if ((uxBits & ALL_HEARTBEATS) != ALL_HEARTBEATS) {
            HAL_Relay_Trip();
            HAL_Alarm_Set(true);

            if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(2U)) == pdTRUE) {
                g_xSharedTelemetry.fault_mask |= FAULT_WATCHDOG_TRIP;
                xSemaphoreGive(xSensorMutex);
            }
        }
    }
}

/* ------------------------------------------------------------
 * Industrial System Initialization & Task Spawning
 * ------------------------------------------------------------ */
bool bInitIndustrialProtectionSystem(void)
{
    xSensorMutex = xSemaphoreCreateMutex();
    if (xSensorMutex == NULL) return false;

    xADC_EmergencyTripSemaphore = xSemaphoreCreateBinary();
    if (xADC_EmergencyTripSemaphore == NULL) return false;

    xTelemetryQueue = xQueueCreate(16U, sizeof(TelemetryData_t));
    if (xTelemetryQueue == NULL) return false;

    xTaskHeartbeatEventGroup = xEventGroupCreate();
    if (xTaskHeartbeatEventGroup == NULL) return false;

    /* Assign priorities matching strict industrial safety hierarchy:
     * vSafetyWatchdogTask        (Priority 6 - Highest System Guardian)
     * vHighSpeedADCProtectionTask (Priority 5 - 100Hz Fast Trip)
     * vProcessingProtectionTask  (Priority 4 - 20Hz Health Processing)
     * vI2CTemperatureTask        (Priority 3 - 5Hz Thermal Monitoring)
     * vIndustrialCANTask         (Priority 2 - 10Hz Bus Broadcast)
     */
    BaseType_t s1 = xTaskCreate(vSafetyWatchdogTask,         "Watchdog", 512, NULL, 6, NULL);
    BaseType_t s2 = xTaskCreate(vHighSpeedADCProtectionTask, "ADC_Prot",  512, NULL, 5, NULL);
    BaseType_t s3 = xTaskCreate(vProcessingProtectionTask,  "Proc_Prot", 512, NULL, 4, NULL);
    BaseType_t s4 = xTaskCreate(vI2CTemperatureTask,        "Temp_Mon",  512, NULL, 3, NULL);
    BaseType_t s5 = xTaskCreate(vIndustrialCANTask,         "CAN_Bus",   512, NULL, 2, NULL);

    return (s1 == pdPASS && s2 == pdPASS && s3 == pdPASS && s4 == pdPASS && s5 == pdPASS);
}

int main(void)
{
    if (bInitIndustrialProtectionSystem()) {
        vTaskStartScheduler();
    }

    while (1);
    return 0;
}
