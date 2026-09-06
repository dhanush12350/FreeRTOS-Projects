#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define MOTOR_COUNT 12U
#define LEG_COUNT 4U
#define MOTORS_PER_LEG (MOTOR_COUNT / LEG_COUNT)
#define ESC_MIN_PULSE_US 1000U
#define ESC_MAX_PULSE_US 2000U
#define ESC_ARM_PULSE_US 1000U
#define MOTOR_COMMAND_TIMEOUT_MS 250U
#define FLIGHT_CONTROL_PERIOD_MS 20U
#define MOTOR_CONTROL_PERIOD_MS 5U
#define TELEMETRY_PERIOD_MS 10U

#define MOTOR_M1 0U
#define MOTOR_M2 1U
#define MOTOR_M3 2U
#define MOTOR_M4 3U
#define MOTOR_M5 4U
#define MOTOR_M6 5U
#define MOTOR_M7 6U
#define MOTOR_M8 7U
#define MOTOR_M9 8U
#define MOTOR_M10 9U
#define MOTOR_M11 10U
#define MOTOR_M12 11U

static const uint8_t xMotorPriorityOrder[MOTOR_COUNT] = {
    MOTOR_M1, MOTOR_M2, MOTOR_M3,
    MOTOR_M4, MOTOR_M5, MOTOR_M6,
    MOTOR_M7, MOTOR_M8, MOTOR_M9,
    MOTOR_M10, MOTOR_M11, MOTOR_M12
};

typedef struct {
    uint8_t timer_index;
    uint8_t timer_channel;
} ESC_PWMMap_t;

typedef struct {
    uint16_t pulse_us[MOTOR_COUNT];
} MotorCommandVector_t;

typedef struct {
    uint8_t motor_id;
    uint16_t rpm;
    int16_t current_ma;
    int8_t temp_c;
    uint8_t fault;
} MotorTelemetry_t;

static QueueHandle_t xMotorTelemetryQueue = NULL;
static SemaphoreHandle_t xMotorCommandMutex = NULL;
static MotorCommandVector_t xLatestMotorCommand = { {ESC_MIN_PULSE_US, ESC_MIN_PULSE_US, ESC_MIN_PULSE_US,
                                                     ESC_MIN_PULSE_US, ESC_MIN_PULSE_US, ESC_MIN_PULSE_US,
                                                     ESC_MIN_PULSE_US, ESC_MIN_PULSE_US, ESC_MIN_PULSE_US,
                                                     ESC_MIN_PULSE_US, ESC_MIN_PULSE_US, ESC_MIN_PULSE_US} };
static volatile TickType_t xLastMotorCommandUpdateTick = 0U;

static const ESC_PWMMap_t xESC_PWMMap[MOTOR_COUNT] = {
    { 0U, 0U }, { 0U, 1U }, { 0U, 2U },
    { 0U, 3U }, { 1U, 0U }, { 1U, 1U },
    { 1U, 2U }, { 1U, 3U }, { 2U, 0U },
    { 2U, 1U }, { 2U, 2U }, { 2U, 3U }
};

void ESC_SetPWM(uint8_t motor_id, uint16_t pulse_us);
void Motor_EmergencyStop(void);
static void vMotorFaultHandler(const char *pcFaultName, uint8_t motor_id, uint16_t pulse_us);

static bool xMotorIdIsValid(uint8_t motor_id)
{
    return (motor_id < MOTOR_COUNT);
}

static uint16_t usClampMotorPulse(uint8_t motor_id, uint16_t pulse_us)
{
    if (!xMotorIdIsValid(motor_id)) {
        return ESC_MIN_PULSE_US;
    }

    if (pulse_us < ESC_MIN_PULSE_US) {
        return ESC_MIN_PULSE_US;
    }

    if (pulse_us > ESC_MAX_PULSE_US) {
        return ESC_MAX_PULSE_US;
    }

    return pulse_us;
}

static void vMotorFaultHandler(const char *pcFaultName, uint8_t motor_id, uint16_t pulse_us)
{
    (void)pcFaultName;
    (void)motor_id;
    (void)pulse_us;
    Motor_EmergencyStop();
}

static void vApplyCommandVectorInPriorityOrder(const MotorCommandVector_t *pxCommand)
{
    if (pxCommand == NULL) {
        return;
    }

    for (uint8_t idx = 0U; idx < MOTOR_COUNT; idx++) {
        uint8_t motor_id = xMotorPriorityOrder[idx];
        uint16_t pulse_us = usClampMotorPulse(motor_id, pxCommand->pulse_us[motor_id]);
        ESC_SetPWM(motor_id, pulse_us);
    }
}

static void UART_ReadMotorTelemetry(uint8_t *buf, uint16_t len)
{
    if (buf != NULL && len > 0U) {
        memset(buf, 0, len);
    }
}

void ESC_SetPWM(uint8_t motor_id, uint16_t pulse_us)
{
    if (!xMotorIdIsValid(motor_id)) {
        return;
    }

    pulse_us = usClampMotorPulse(motor_id, pulse_us);

    (void)xESC_PWMMap;
    (void)motor_id;
    (void)pulse_us;
}

void Motor_EmergencyStop(void)
{
    if (xMotorCommandMutex != NULL) {
        if (xSemaphoreTake(xMotorCommandMutex, pdMS_TO_TICKS(5U)) == pdTRUE) {
            for (uint8_t motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++) {
                xLatestMotorCommand.pulse_us[motor_id] = ESC_MIN_PULSE_US;
            }
            xLastMotorCommandUpdateTick = xTaskGetTickCount();
            xSemaphoreGive(xMotorCommandMutex);
        } else {
            for (uint8_t motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++) {
                xLatestMotorCommand.pulse_us[motor_id] = ESC_MIN_PULSE_US;
            }
            xLastMotorCommandUpdateTick = xTaskGetTickCount();
        }
    }

    for (uint8_t motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++) {
        ESC_SetPWM(motor_id, ESC_MIN_PULSE_US);
    }
}

static void vFlightControlTask(void *pvParameters)
{
    (void)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    MotorCommandVector_t xDesiredCommand;

    for (uint8_t motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++) {
        xDesiredCommand.pulse_us[motor_id] = ESC_ARM_PULSE_US;
    }

    while (1) {
        for (uint8_t motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++) {
            xDesiredCommand.pulse_us[motor_id] = 1100U + (uint16_t)(motor_id * 25U);
        }

        if (xMotorCommandMutex != NULL) {
            if (xSemaphoreTake(xMotorCommandMutex, pdMS_TO_TICKS(10U)) == pdTRUE) {
                for (uint8_t motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++) {
                    xLatestMotorCommand.pulse_us[motor_id] = xDesiredCommand.pulse_us[motor_id];
                }
                xLastMotorCommandUpdateTick = xTaskGetTickCount();
                xSemaphoreGive(xMotorCommandMutex);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(FLIGHT_CONTROL_PERIOD_MS));
    }
}

static void vMotorControlTask(void *pvParameters)
{
    (void)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    MotorCommandVector_t xCurrentCommand;

    for (uint8_t motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++) {
        xLatestMotorCommand.pulse_us[motor_id] = ESC_MIN_PULSE_US;
        ESC_SetPWM(motor_id, ESC_MIN_PULSE_US);
    }

    xLastMotorCommandUpdateTick = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS));

        if (xMotorCommandMutex != NULL) {
            if (xSemaphoreTake(xMotorCommandMutex, pdMS_TO_TICKS(2U)) == pdTRUE) {
                for (uint8_t motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++) {
                    xCurrentCommand.pulse_us[motor_id] = xLatestMotorCommand.pulse_us[motor_id];
                }
                xSemaphoreGive(xMotorCommandMutex);
            }
        }

        if ((xTaskGetTickCount() - xLastMotorCommandUpdateTick) > pdMS_TO_TICKS(MOTOR_COMMAND_TIMEOUT_MS)) {
            Motor_EmergencyStop();
        } else {
            vApplyCommandVectorInPriorityOrder(&xCurrentCommand);
        }
    }
}

static void vMotorTelemetryTask(void *pvParameters)
{
    (void)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t rxBuf[32];

    while (1) {
        UART_ReadMotorTelemetry(rxBuf, sizeof(rxBuf));

        if (rxBuf[0] != 0U) {
            MotorTelemetry_t telemetry;
            telemetry.motor_id = rxBuf[1] % MOTOR_COUNT;
            telemetry.rpm = (uint16_t)(rxBuf[2] | (rxBuf[3] << 8));
            telemetry.current_ma = (int16_t)(rxBuf[4] | (rxBuf[5] << 8));
            telemetry.temp_c = (int8_t)rxBuf[6];
            telemetry.fault = rxBuf[7];

            if (xMotorTelemetryQueue != NULL) {
                if (xQueueSend(xMotorTelemetryQueue, &telemetry, 0U) != pdTRUE) {
                    MotorTelemetry_t dummy;
                    xQueueReceive(xMotorTelemetryQueue, &dummy, 0U);
                    xQueueSend(xMotorTelemetryQueue, &telemetry, 0U);
                }
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}

bool bAppInitMotorCommunication(void)
{
    xMotorTelemetryQueue = xQueueCreate(16U, sizeof(MotorTelemetry_t));
    if (xMotorTelemetryQueue == NULL) {
        return false;
    }

    xMotorCommandMutex = xSemaphoreCreateMutex();
    if (xMotorCommandMutex == NULL) {
        return false;
    }

    BaseType_t status1 = xTaskCreate(vMotorControlTask,   "MotorCtrl",  512, NULL, 4, NULL);
    BaseType_t status2 = xTaskCreate(vFlightControlTask,  "FlightCtrl", 512, NULL, 3, NULL);
    BaseType_t status3 = xTaskCreate(vMotorTelemetryTask, "MotorTelem", 512, NULL, 2, NULL);

    return (status1 == pdPASS && status2 == pdPASS && status3 == pdPASS);
}

void Motor_SendSingleCommand(uint8_t motor_id, uint16_t pulse_us)
{
    if (!xMotorIdIsValid(motor_id)) {
        return;
    }

    pulse_us = usClampMotorPulse(motor_id, pulse_us);

    if (xMotorCommandMutex != NULL) {
        if (xSemaphoreTake(xMotorCommandMutex, pdMS_TO_TICKS(10U)) == pdTRUE) {
            xLatestMotorCommand.pulse_us[motor_id] = pulse_us;
            xLastMotorCommandUpdateTick = xTaskGetTickCount();
            xSemaphoreGive(xMotorCommandMutex);
        }
    }
}

void Motor_SendAllCommands(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4,
                           uint16_t m5, uint16_t m6, uint16_t m7, uint16_t m8,
                           uint16_t m9, uint16_t m10, uint16_t m11, uint16_t m12)
{
    MotorCommandVector_t xCommand = { { m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12 } };

    if (xMotorCommandMutex != NULL) {
        if (xSemaphoreTake(xMotorCommandMutex, pdMS_TO_TICKS(10U)) == pdTRUE) {
            for (uint8_t motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++) {
                xLatestMotorCommand.pulse_us[motor_id] = usClampMotorPulse(motor_id, xCommand.pulse_us[motor_id]);
            }
            xLastMotorCommandUpdateTick = xTaskGetTickCount();
            xSemaphoreGive(xMotorCommandMutex);
        }
    }
}

void Motor_SendMovementVector(const uint16_t pulses[MOTOR_COUNT])
{
    if (pulses == NULL) {
        return;
    }

    if (xMotorCommandMutex != NULL) {
        if (xSemaphoreTake(xMotorCommandMutex, pdMS_TO_TICKS(10U)) == pdTRUE) {
            for (uint8_t motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++) {
                xLatestMotorCommand.pulse_us[motor_id] = usClampMotorPulse(motor_id, pulses[motor_id]);
            }
            xLastMotorCommandUpdateTick = xTaskGetTickCount();
            xSemaphoreGive(xMotorCommandMutex);
        }
    }
}

int main(void)
{
    if (bAppInitMotorCommunication()) {
        vTaskStartScheduler();
    }

    while (1);
    return 0;
}
