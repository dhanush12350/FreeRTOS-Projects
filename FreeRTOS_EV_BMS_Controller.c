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
 * EV Battery Management System (BMS) Safety Thresholds
 * ------------------------------------------------------------ */
#define CELL_COUNT                 8U
#define TEMP_SENSOR_COUNT          4U

#define CELL_MAX_VOLTAGE_MV     4200U   /* 4.2V Max Overvoltage Limit */
#define CELL_MIN_VOLTAGE_MV     3000U   /* 3.0V Min Undervoltage Limit */
#define CELL_BALANCE_TARGET_MV     5U   /* 5mV Max Allowable Imbalance */
#define MAX_DISCHARGE_CURRENT_MA 100000 /* 100A Overcurrent Limit */
#define MAX_PACK_TEMP_C          60.0f  /* 60°C Overtemperature Limit */

#define VOLTAGE_TASK_PERIOD_MS    10U   /* 100Hz Fast Sampling */
#define THERMAL_TASK_PERIOD_MS   100U   /* 10Hz Thermal Sampling */
#define BALANCING_TASK_PERIOD_MS1000U   /* 1Hz Passive Cell Balancing */
#define CAN_TELEMETRY_PERIOD_MS   50U   /* 20Hz Vehicle Bus Broadcast */
#define BMS_WATCHDOG_PERIOD_MS   500U   /* 500ms Watchdog Check */

#define HEARTBEAT_VOLTAGE_BIT   ( 1 << 0 )
#define HEARTBEAT_THERMAL_BIT   ( 1 << 1 )
#define HEARTBEAT_BALANCING_BIT ( 1 << 2 )
#define ALL_BMS_HEARTBEATS      ( HEARTBEAT_VOLTAGE_BIT | HEARTBEAT_THERMAL_BIT | HEARTBEAT_BALANCING_BIT )

#define BMS_FAULT_NONE          0x00U
#define FAULT_OVERVOLTAGE       0x01U
#define FAULT_UNDERVOLTAGE      0x02U
#define FAULT_OVERCURRENT       0x04U
#define FAULT_OVERTEMP          0x08U
#define FAULT_SHORT_CIRCUIT     0x10U
#define FAULT_WATCHDOG_TRIP     0x20U

/* ------------------------------------------------------------
 * BMS Structures
 * ------------------------------------------------------------ */
typedef struct {
    uint16_t cell_voltages_mv[CELL_COUNT];
    int32_t  pack_current_ma;
    float    temperatures_c[TEMP_SENSOR_COUNT];
    float    state_of_charge_soc;
    uint32_t fault_code;
    bool     contactors_closed;
} BMSPackStatus_t;

typedef struct {
    uint32_t can_id;
    uint8_t  length;
    uint8_t  payload[8];
} CANMessage_t;

/* ------------------------------------------------------------
 * RTOS Primitives & Global Handles
 * ------------------------------------------------------------ */
static SemaphoreHandle_t  xBMSDataMutex                = NULL;
static SemaphoreHandle_t  xShortCircuitISR_Semaphore   = NULL;
static QueueHandle_t      xCANTelemetryQueue           = NULL;
static EventGroupHandle_t xBMSWatchdogEventGroup       = NULL;

static BMSPackStatus_t g_xBMSStatus = {
    {3700U, 3705U, 3698U, 3702U, 3701U, 3699U, 3704U, 3700U},
    15000,
    {28.5f, 29.0f, 28.2f, 28.8f},
    78.5f,
    BMS_FAULT_NONE,
    true
};

/* ------------------------------------------------------------
 * Hardware Abstraction Layer (HAL) Mocks
 * ------------------------------------------------------------ */
static void HAL_BMS_OpenMainContactors(void)
{
    g_xBMSStatus.contactors_closed = false;
    /* Hardware High-Voltage Relay Disconnect (HV+ and HV- Contactors) */
}

static void HAL_BMS_SetCellBalancingBleed(uint8_t cell_index, bool enable)
{
    (void)cell_index;
    (void)enable;
    /* Enable passive MOSFET bleed resistor for target cell */
}

static void HAL_BMS_SetCoolingFanPWM(uint8_t duty_cycle_percent)
{
    (void)duty_cycle_percent;
    /* Set cooling fan PWM speed */
}

static void HAL_CAN_TransmitMessage(const CANMessage_t *pxMsg)
{
    (void)pxMsg;
    /* Write CAN frame to hardware transmission mailbox */
}

/* ------------------------------------------------------------
 * Short-Circuit Hardware Comparator ISR (Sub-Microsecond)
 * ------------------------------------------------------------ */
void EXTI_ShortCircuit_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Instantly open high-voltage contactors at hardware level */
    HAL_BMS_OpenMainContactors();

    if (xShortCircuitISR_Semaphore != NULL) {
        xSemaphoreGiveFromISR(xShortCircuitISR_Semaphore, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ------------------------------------------------------------
 * Task 1: High-Speed Short Circuit Protection Task (Priority 6)
 * Handles emergency disconnects triggered by hardware ISR
 * ------------------------------------------------------------ */
static void vHighSpeedShortCircuitProtectionTask(void *pvParameters)
{
    (void)pvParameters;

    while (1) {
        if (xSemaphoreTake(xShortCircuitISR_Semaphore, portMAX_DELAY) == pdTRUE) {
            if (xSemaphoreTake(xBMSDataMutex, pdMS_TO_TICKS(2U)) == pdTRUE) {
                g_xBMSStatus.fault_code |= FAULT_SHORT_CIRCUIT;
                xSemaphoreGive(xBMSDataMutex);
            }
            HAL_BMS_OpenMainContactors();
        }
    }
}

/* ------------------------------------------------------------
 * Task 2: Cell Voltage & Current Sampling Task (Priority 5 - 100Hz)
 * Reads cell voltages, measures current, performs Coulomb Counting SoC
 * ------------------------------------------------------------ */
static void vCellVoltageCurrentTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(VOLTAGE_TASK_PERIOD_MS));

        uint32_t ulLocalFault = BMS_FAULT_NONE;
        int32_t  iCurrentMa   = 22000; /* Simulated 22A discharge */

        if (iCurrentMa > MAX_DISCHARGE_CURRENT_MA) {
            ulLocalFault |= FAULT_OVERCURRENT;
            HAL_BMS_OpenMainContactors();
        }

        if (xSemaphoreTake(xBMSDataMutex, pdMS_TO_TICKS(2U)) == pdTRUE) {
            for (uint8_t i = 0U; i < CELL_COUNT; i++) {
                if (g_xBMSStatus.cell_voltages_mv[i] > CELL_MAX_VOLTAGE_MV) {
                    ulLocalFault |= FAULT_OVERVOLTAGE;
                } else if (g_xBMSStatus.cell_voltages_mv[i] < CELL_MIN_VOLTAGE_MV) {
                    ulLocalFault |= FAULT_UNDERVOLTAGE;
                }
            }

            g_xBMSStatus.pack_current_ma = iCurrentMa;
            g_xBMSStatus.fault_code     |= ulLocalFault;

            /* Coulomb Counting SoC Integration: SoC = SoC_prev - (I * dt) */
            g_xBMSStatus.state_of_charge_soc -= ((float)iCurrentMa / 3600000.0f) * (VOLTAGE_TASK_PERIOD_MS / 1000.0f);

            if (ulLocalFault != BMS_FAULT_NONE) {
                HAL_BMS_OpenMainContactors();
            }

            xSemaphoreGive(xBMSDataMutex);
        }

        if (xBMSWatchdogEventGroup != NULL) {
            xEventGroupSetBits(xBMSWatchdogEventGroup, HEARTBEAT_VOLTAGE_BIT);
        }
    }
}

/* ------------------------------------------------------------
 * Task 3: Thermal Management Task (Priority 4 - 10Hz)
 * Monitors battery pack thermistors & controls cooling fan PWM
 * ------------------------------------------------------------ */
static void vThermalManagementTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(THERMAL_TASK_PERIOD_MS));

        float fMaxTemp = 0.0f;
        uint32_t ulLocalFault = BMS_FAULT_NONE;

        if (xSemaphoreTake(xBMSDataMutex, pdMS_TO_TICKS(5U)) == pdTRUE) {
            for (uint8_t i = 0U; i < TEMP_SENSOR_COUNT; i++) {
                if (g_xBMSStatus.temperatures_c[i] > fMaxTemp) {
                    fMaxTemp = g_xBMSStatus.temperatures_c[i];
                }
            }

            if (fMaxTemp > MAX_PACK_TEMP_C) {
                ulLocalFault |= FAULT_OVERTEMP;
                g_xBMSStatus.fault_code |= ulLocalFault;
                HAL_BMS_OpenMainContactors();
            }

            xSemaphoreGive(xBMSDataMutex);
        }

        /* Proportional Thermal Fan Speed Control */
        if (fMaxTemp > 45.0f) {
            HAL_BMS_SetCoolingFanPWM(100U);
        } else if (fMaxTemp > 35.0f) {
            HAL_BMS_SetCoolingFanPWM(50U);
        } else {
            HAL_BMS_SetCoolingFanPWM(0U);
        }

        if (xBMSWatchdogEventGroup != NULL) {
            xEventGroupSetBits(xBMSWatchdogEventGroup, HEARTBEAT_THERMAL_BIT);
        }
    }
}

/* ------------------------------------------------------------
 * Task 4: Cell Balancing Task (Priority 3 - 1Hz)
 * Active/Passive cell balancing to equalize voltages across cells
 * ------------------------------------------------------------ */
static void vCellBalancingTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(BALANCING_TASK_PERIOD_MS));

        uint16_t uMinVoltage = 0xFFFFU;
        uint16_t uLocalVoltages[CELL_COUNT];

        if (xSemaphoreTake(xBMSDataMutex, pdMS_TO_TICKS(5U)) == pdTRUE) {
            for (uint8_t i = 0U; i < CELL_COUNT; i++) {
                uLocalVoltages[i] = g_xBMSStatus.cell_voltages_mv[i];
                if (uLocalVoltages[i] < uMinVoltage) {
                    uMinVoltage = uLocalVoltages[i];
                }
            }
            xSemaphoreGive(xBMSDataMutex);
        }

        /* Enable bleed resistor for cells exceeding minimum by target threshold */
        for (uint8_t i = 0U; i < CELL_COUNT; i++) {
            if ((uLocalVoltages[i] - uMinVoltage) > CELL_BALANCE_TARGET_MV) {
                HAL_BMS_SetCellBalancingBleed(i, true);
            } else {
                HAL_BMS_SetCellBalancingBleed(i, false);
            }
        }

        if (xBMSWatchdogEventGroup != NULL) {
            xEventGroupSetBits(xBMSWatchdogEventGroup, HEARTBEAT_BALANCING_BIT);
        }
    }
}

/* ------------------------------------------------------------
 * Task 5: Vehicle CAN Bus Telemetry Task (Priority 2 - 20Hz)
 * Broadcasts J1939 / CANopen BMS telemetry frames to VCU
 * ------------------------------------------------------------ */
static void vCANBusTelemetryTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    BMSPackStatus_t xLocalStatus;
    CANMessage_t xMsg;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CAN_TELEMETRY_PERIOD_MS));

        if (xSemaphoreTake(xBMSDataMutex, pdMS_TO_TICKS(5U)) == pdTRUE) {
            xLocalStatus = g_xBMSStatus;
            xSemaphoreGive(xBMSDataMutex);
        }

        /* CAN Frame 1: Pack Summary (ID: 0x18FF0100) */
        xMsg.can_id = 0x18FF0100U;
        xMsg.length = 8U;
        
        uint16_t current_scaled = (uint16_t)((xLocalStatus.pack_current_ma / 100) + 1000);
        uint8_t  soc_byte       = (uint8_t)(xLocalStatus.state_of_charge_soc * 2.0f);

        xMsg.payload[0] = (uint8_t)(current_scaled & 0xFF);
        xMsg.payload[1] = (uint8_t)((current_scaled >> 8) & 0xFF);
        xMsg.payload[2] = soc_byte;
        xMsg.payload[3] = (uint8_t)(xLocalStatus.fault_code & 0xFF);
        xMsg.payload[4] = xLocalStatus.contactors_closed ? 0x01U : 0x00U;
        xMsg.payload[5] = 0x00U;
        xMsg.payload[6] = 0x00U;
        xMsg.payload[7] = 0xAAU;

        HAL_CAN_TransmitMessage(&xMsg);
    }
}

/* ------------------------------------------------------------
 * Task 6: Ultimate BMS Safety Watchdog Task (Priority 7 - Highest)
 * Ensures zero-deadlock health monitoring across all battery tasks
 * ------------------------------------------------------------ */
static void vBMSWatchdogTask(void *pvParameters)
{
    (void)pvParameters;

    while (1) {
        EventBits_t uxBits = xEventGroupWaitBits(
            xBMSWatchdogEventGroup,
            ALL_BMS_HEARTBEATS,
            pdTRUE,
            pdTRUE,
            pdMS_TO_TICKS(BMS_WATCHDOG_PERIOD_MS)
        );

        if ((uxBits & ALL_BMS_HEARTBEATS) != ALL_BMS_HEARTBEATS) {
            HAL_BMS_OpenMainContactors();

            if (xSemaphoreTake(xBMSDataMutex, pdMS_TO_TICKS(2U)) == pdTRUE) {
                g_xBMSStatus.fault_code |= FAULT_WATCHDOG_TRIP;
                xSemaphoreGive(xBMSDataMutex);
            }
        }
    }
}

/* ------------------------------------------------------------
 * BMS System Initialization & Task Spawning
 * ------------------------------------------------------------ */
bool bInitEVBatteryManagementSystem(void)
{
    xBMSDataMutex = xSemaphoreCreateMutex();
    if (xBMSDataMutex == NULL) return false;

    xShortCircuitISR_Semaphore = xSemaphoreCreateBinary();
    if (xShortCircuitISR_Semaphore == NULL) return false;

    xCANTelemetryQueue = xQueueCreate(16U, sizeof(CANMessage_t));
    if (xCANTelemetryQueue == NULL) return false;

    xBMSWatchdogEventGroup = xEventGroupCreate();
    if (xBMSWatchdogEventGroup == NULL) return false;

    /* Assign priorities according to automotive safety criticality:
     * vBMSWatchdogTask                    (Priority 7 - Highest System Guardian)
     * vHighSpeedShortCircuitProtectionTask (Priority 6 - Instant Fast Trip)
     * vCellVoltageCurrentTask             (Priority 5 - 100Hz Coulomb Counting & Protection)
     * vThermalManagementTask              (Priority 4 - 10Hz Fan PWM Control)
     * vCellBalancingTask                  (Priority 3 - 1Hz Passive Bleed Balancing)
     * vCANBusTelemetryTask                (Priority 2 - 20Hz Vehicle Bus Broadcast)
     */
    BaseType_t s1 = xTaskCreate(vBMSWatchdogTask,                    "BMS_Watchdog", 512, NULL, 7, NULL);
    BaseType_t s2 = xTaskCreate(vHighSpeedShortCircuitProtectionTask,"ShortCircuit",512, NULL, 6, NULL);
    BaseType_t s3 = xTaskCreate(vCellVoltageCurrentTask,            "VoltageCurr",  512, NULL, 5, NULL);
    BaseType_t s4 = xTaskCreate(vThermalManagementTask,             "ThermalCtrl",  512, NULL, 4, NULL);
    BaseType_t s5 = xTaskCreate(vCellBalancingTask,                 "Balancing",    512, NULL, 3, NULL);
    BaseType_t s6 = xTaskCreate(vCANBusTelemetryTask,                "CAN_Telemetry",512, NULL, 2, NULL);

    return (s1 == pdPASS && s2 == pdPASS && s3 == pdPASS && s4 == pdPASS && s5 == pdPASS && s6 == pdPASS);
}

int main(void)
{
    if (bInitEVBatteryManagementSystem()) {
        vTaskStartScheduler();
    }

    while (1);
    return 0;
}
