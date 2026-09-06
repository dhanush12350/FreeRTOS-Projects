#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------
 * System & Timing Constants
 * ------------------------------------------------------------ */
#define MOTOR_CONTROL_PERIOD_MS   20U
#define SAFETY_CHECK_PERIOD_MS    50U
#define COMMAND_TIMEOUT_MS       500U
#define MAX_PWM_DUTY            1000U
#define ENCODER_PULSES_PER_REV    360U

/* ------------------------------------------------------------
 * Data Structures
 * ------------------------------------------------------------ */
typedef struct {
    int16_t left_target_rpm;
    int16_t right_target_rpm;
} MotorCommand_t;

typedef struct {
    int16_t left_current_rpm;
    int16_t right_current_rpm;
    uint32_t left_total_pulses;
    uint32_t right_total_pulses;
} EncoderData_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float prev_error;
    float integral;
} PIDController_t;

/* ------------------------------------------------------------
 * FreeRTOS Handles & Global State
 * ------------------------------------------------------------ */
static QueueHandle_t     xMotorQueue       = NULL;
static SemaphoreHandle_t xEncoderMutex     = NULL;
static SemaphoreHandle_t xUARTMutex        = NULL;
static TaskHandle_t      xEncoderTaskHandle = NULL;

static EncoderData_t     g_xEncoderData             = {0};
static volatile uint32_t g_ulLastCommandTick        = 0U;
static volatile bool     g_bEmergencyStopTriggered = false;

/* ------------------------------------------------------------
 * Hardware Abstraction Layer (HAL) Mock Functions for RP2040
 * ------------------------------------------------------------ */
static void PWM_SetLeftDuty(uint16_t duty)
{
    if (duty > MAX_PWM_DUTY) duty = MAX_PWM_DUTY;
    (void)duty;
}

static void PWM_SetRightDuty(uint16_t duty)
{
    if (duty > MAX_PWM_DUTY) duty = MAX_PWM_DUTY;
    (void)duty;
}

static void System_Log(const char *pcMsg)
{
    if (xUARTMutex != NULL) {
        if (xSemaphoreTake(xUARTMutex, pdMS_TO_TICKS(5U)) == pdTRUE) {
            (void)pcMsg;
            xSemaphoreGive(xUARTMutex);
        }
    }
}

/* ------------------------------------------------------------
 * PID Calculation Function
 * ------------------------------------------------------------ */
static float PID_Compute(PIDController_t *pid, float setpoint, float measured)
{
    float error = setpoint - measured;
    pid->integral += error * (MOTOR_CONTROL_PERIOD_MS / 1000.0f);
    float derivative = (error - pid->prev_error) / (MOTOR_CONTROL_PERIOD_MS / 1000.0f);
    pid->prev_error = error;

    float output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    if (output > (float)MAX_PWM_DUTY) output = (float)MAX_PWM_DUTY;
    if (output < 0.0f) output = 0.0f;
    return output;
}

/* ------------------------------------------------------------
 * GPIO Encoder Interrupt Service Routine (RP2040)
 * ------------------------------------------------------------ */
void gpio_encoder_isr(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (xEncoderTaskHandle != NULL) {
        vTaskNotifyGiveFromISR(xEncoderTaskHandle, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ------------------------------------------------------------
 * Task 1: Command Task (Priority 2)
 * Receives velocity commands over UART/USB and queues them
 * ------------------------------------------------------------ */
static void vCommandTask(void *pvParameters)
{
    (void)pvParameters;
    MotorCommand_t xCmd;

    while (1) {
        /* Simulate receiving command: e.g., FORWARD at 120 RPM */
        xCmd.left_target_rpm  = 120;
        xCmd.right_target_rpm = 120;

        if (xMotorQueue != NULL) {
            if (xQueueSend(xMotorQueue, &xCmd, pdMS_TO_TICKS(10U)) == pdTRUE) {
                g_ulLastCommandTick = xTaskGetTickCount();
                System_Log("Command Sent: Left=120 RPM, Right=120 RPM");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100U));
    }
}

/* ------------------------------------------------------------
 * Task 2: Encoder Task (Priority 3)
 * Woken by Task Notifications from Encoder ISR
 * ------------------------------------------------------------ */
static void vEncoderTask(void *pvParameters)
{
    (void)pvParameters;

    while (1) {
        /* Block until woken by encoder pulse ISR notification */
        uint32_t ulNotificationCount = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100U));

        if (ulNotificationCount > 0U) {
            if (xSemaphoreTake(xEncoderMutex, pdMS_TO_TICKS(5U)) == pdTRUE) {
                g_xEncoderData.left_total_pulses  += ulNotificationCount;
                g_xEncoderData.right_total_pulses += ulNotificationCount;

                /* Compute velocity / RPM */
                g_xEncoderData.left_current_rpm  = (int16_t)((g_xEncoderData.left_total_pulses * 60) / ENCODER_PULSES_PER_REV);
                g_xEncoderData.right_current_rpm = (int16_t)((g_xEncoderData.right_total_pulses * 60) / ENCODER_PULSES_PER_REV);

                xSemaphoreGive(xEncoderMutex);
            }
        }
    }
}

/* ------------------------------------------------------------
 * Task 3: Motor Control Task (Priority 4 - Periodic 50Hz / 20ms)
 * Runs PID speed regulation loop and updates PWM duty cycles
 * ------------------------------------------------------------ */
static void vMotorControlTask(void *pvParameters)
{
    (void)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    MotorCommand_t xCurrentCmd = {0, 0};
    EncoderData_t  xLocalEncoder = {0, 0, 0, 0};

    PIDController_t pidLeft  = {1.5f, 0.2f, 0.05f, 0.0f, 0.0f};
    PIDController_t pidRight = {1.5f, 0.2f, 0.05f, 0.0f, 0.0f};

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS));

        /* Check for new command from queue */
        if (xMotorQueue != NULL) {
            xQueueReceive(xMotorQueue, &xCurrentCmd, 0U);
        }

        /* Safely read encoder feedback using Mutex */
        if (xSemaphoreTake(xEncoderMutex, pdMS_TO_TICKS(2U)) == pdTRUE) {
            xLocalEncoder = g_xEncoderData;
            xSemaphoreGive(xEncoderMutex);
        }

        /* If safety stop is active, kill PWM */
        if (g_bEmergencyStopTriggered) {
            PWM_SetLeftDuty(0U);
            PWM_SetRightDuty(0U);
            continue;
        }

        /* Execute PID speed control for Left and Right motors */
        float leftPwm  = PID_Compute(&pidLeft,  (float)xCurrentCmd.left_target_rpm,  (float)xLocalEncoder.left_current_rpm);
        float rightPwm = PID_Compute(&pidRight, (float)xCurrentCmd.right_target_rpm, (float)xLocalEncoder.right_current_rpm);

        PWM_SetLeftDuty((uint16_t)leftPwm);
        PWM_SetRightDuty((uint16_t)rightPwm);
    }
}

/* ------------------------------------------------------------
 * Task 4: Safety Task (Priority 5 - Highest System Priority)
 * Monitors communication timeout & hardware faults
 * ------------------------------------------------------------ */
static void vSafetyTask(void *pvParameters)
{
    (void)pvParameters;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(SAFETY_CHECK_PERIOD_MS));

        TickType_t xCurrentTick = xTaskGetTickCount();

        /* Check if command timeout occurred (>500ms since last valid command) */
        if ((xCurrentTick - g_ulLastCommandTick) > pdMS_TO_TICKS(COMMAND_TIMEOUT_MS)) {
            g_bEmergencyStopTriggered = true;
            PWM_SetLeftDuty(0U);
            PWM_SetRightDuty(0U);
            System_Log("SAFETY WARNING: Communication Timeout! Motors Stopped.");
        } else {
            g_bEmergencyStopTriggered = false;
        }
    }
}

/* ------------------------------------------------------------
 * System Initialization & Task Spawning
 * ------------------------------------------------------------ */
bool bInitRobotControlSystem(void)
{
    xMotorQueue = xQueueCreate(8U, sizeof(MotorCommand_t));
    if (xMotorQueue == NULL) return false;

    xEncoderMutex = xSemaphoreCreateMutex();
    if (xEncoderMutex == NULL) return false;

    xUARTMutex = xSemaphoreCreateMutex();
    if (xUARTMutex == NULL) return false;

    /* Assign priorities matching rate-monotonic deadlines:
     * vSafetyTask        (Priority 5 - Highest)
     * vMotorControlTask  (Priority 4 - 50Hz PID Loop)
     * vEncoderTask       (Priority 3 - ISR Notified)
     * vCommandTask       (Priority 2 - Telemetry/Command Input)
     */
    BaseType_t s1 = xTaskCreate(vSafetyTask,       "SafetyTask",  512, NULL, 5, NULL);
    BaseType_t s2 = xTaskCreate(vMotorControlTask, "MotorCtrl",   512, NULL, 4, NULL);
    BaseType_t s3 = xTaskCreate(vEncoderTask,      "EncoderTask", 512, NULL, 3, &xEncoderTaskHandle);
    BaseType_t s4 = xTaskCreate(vCommandTask,      "CommandTask", 512, NULL, 2, NULL);

    return (s1 == pdPASS && s2 == pdPASS && s3 == pdPASS && s4 == pdPASS);
}

int main(void)
{
    if (bInitRobotControlSystem()) {
        vTaskStartScheduler();
    }

    while (1);
    return 0;
}
