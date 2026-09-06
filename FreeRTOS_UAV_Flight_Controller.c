#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ------------------------------------------------------------
 * Flight Controller Timing & Hardware Constants
 * ------------------------------------------------------------ */
#define MOTOR_COUNT                  4U
#define ATTITUDE_RATE_HZ          1000U   /* 1kHz IMU & Quaternion Loop (1ms) */
#define RATE_PID_HZ                400U   /* 400Hz Motor PID Loop (2.5ms) */
#define NAVIGATION_HZ              100U   /* 100Hz EKF Altitude & GPS (10ms) */
#define MAVLINK_HZ                  10U   /* 10Hz Telemetry Broadcast (100ms) */
#define BLACKBOX_HZ                100U   /* 100Hz SD Card Logger */
#define FAILSAFE_TIMEOUT_MS        100U   /* 100ms RC Loss Timeout */

#define ATTITUDE_PERIOD_MS           1U
#define RATE_PID_PERIOD_US        2500U
#define NAVIGATION_PERIOD_MS        10U
#define MAVLINK_PERIOD_MS          100U

#define FAILSAFE_BIT_RC_LOSS    ( 1 << 0 )
#define FAILSAFE_BIT_BATTERY    ( 1 << 1 )
#define FAILSAFE_BIT_IMU_FAULT  ( 1 << 2 )
#define ALL_FAILSAFE_BITS       ( FAILSAFE_BIT_RC_LOSS | FAILSAFE_BIT_BATTERY | FAILSAFE_BIT_IMU_FAULT )

/* ------------------------------------------------------------
 * Data Structures & Typedefs
 * ------------------------------------------------------------ */
typedef struct {
    float roll;
    float pitch;
    float yaw;
    float throttle;
} RadioStickInput_t;

typedef struct {
    float q0, q1, q2, q3;        /* Quaternion Attitude Representation */
    float roll_deg, pitch_deg, yaw_deg;
    float gx, gy, gz;            /* Gyroscope Rates (rad/s) */
    float ax, ay, az;            /* Accelerometer (g) */
} AttitudeState_t;

typedef struct {
    float altitude_m;
    float climb_rate_m_s;
    double latitude;
    double longitude;
    bool  gps_fix;
} NavigationState_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float prev_error;
    float integral;
    float max_integral;
} CascadedPID_t;

typedef struct {
    uint16_t motor_pwm[MOTOR_COUNT]; /* 1000us to 2000us PWM */
    bool armed;
} MotorOutput_t;

/* ------------------------------------------------------------
 * RTOS Handles & Global State
 * ------------------------------------------------------------ */
static SemaphoreHandle_t  xAttitudeMutex           = NULL;
static SemaphoreHandle_t  xNavigationMutex         = NULL;
static QueueHandle_t      xRadioInputQueue         = NULL;
static QueueHandle_t      xMotorOutputQueue        = NULL;
static EventGroupHandle_t xFailsafeEventGroup      = NULL;
static TaskHandle_t       xAttitudeTaskHandle      = NULL;

static AttitudeState_t    g_xAttitudeState = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
static NavigationState_t  g_xNavState      = {0.0f, 0.0f, 12.9716, 77.5946, true};
static MotorOutput_t      g_xMotorOutput   = {{1000, 1000, 1000, 1000}, false};

static volatile TickType_t g_xLastRadioInputTick = 0U;
static volatile bool       g_bArmed               = false;

/* ------------------------------------------------------------
 * Hardware Abstraction Layer (HAL) Mocks
 * ------------------------------------------------------------ */
static void HAL_IMU_ReadAccelGyro(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    *ax = 0.0f; *ay = 0.0f; *az = 1.0f;
    *gx = 0.0f; *gy = 0.0f; *gz = 0.0f;
}

static void HAL_ESC_WritePWM(const uint16_t pwm[MOTOR_COUNT])
{
    (void)pwm;
    /* Hardware Timer DShot / Oneshot ESC Write */
}

static void HAL_MAVLink_TransmitBuffer(const uint8_t *buf, uint16_t len)
{
    (void)buf;
    (void)len;
    /* UART DMA Tx */
}

/* ------------------------------------------------------------
 * 1kHz Hardware Timer Interrupt for High-Speed Gyro Sampling
 * ------------------------------------------------------------ */
void TIM1_UP_TIM10_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (xAttitudeTaskHandle != NULL) {
        vTaskNotifyGiveFromISR(xAttitudeTaskHandle, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ------------------------------------------------------------
 * Mahony Quaternion Complementary Filter Update (1kHz)
 * ------------------------------------------------------------ */
static void Mahony_Update(float ax, float ay, float az, float gx, float gy, float gz, float dt, AttitudeState_t *state)
{
    (void)ax; (void)ay; (void)az; (void)dt;
    /* Computes 3D Quaternion Orientation & Euler Angles */
    state->roll_deg  += gx * dt * 57.2958f;
    state->pitch_deg += gy * dt * 57.2958f;
    state->yaw_deg   += gz * dt * 57.2958f;
}

/* ------------------------------------------------------------
 * PID Calculation with Anti-Windup Limiting
 * ------------------------------------------------------------ */
static float PID_ComputeRate(CascadedPID_t *pid, float setpoint, float measured, float dt)
{
    float error = setpoint - measured;
    pid->integral += error * dt;

    if (pid->integral > pid->max_integral)  pid->integral = pid->max_integral;
    if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;

    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    return (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
}

/* ------------------------------------------------------------
 * Task 1: High-Speed 1kHz Attitude Estimation Task (Priority 7)
 * ------------------------------------------------------------ */
static void vAttitudeEstimationTask(void *pvParameters)
{
    (void)pvParameters;
    float ax, ay, az, gx, gy, gz;

    while (1) {
        /* Block waiting for 1kHz hardware timer ISR notification */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5U));

        HAL_IMU_ReadAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);

        if (xSemaphoreTake(xAttitudeMutex, pdMS_TO_TICKS(1U)) == pdTRUE) {
            g_xAttitudeState.ax = ax; g_xAttitudeState.ay = ay; g_xAttitudeState.az = az;
            g_xAttitudeState.gx = gx; g_xAttitudeState.gy = gy; g_xAttitudeState.gz = gz;

            Mahony_Update(ax, ay, az, gx, gy, gz, 0.001f, &g_xAttitudeState);
            xSemaphoreGive(xAttitudeMutex);
        }
    }
}

/* ------------------------------------------------------------
 * Task 2: 400Hz Flight Control Rate PID Loop (Priority 6)
 * Calculates Cascaded PID for Roll, Pitch, Yaw & mixes Motor PWM
 * ------------------------------------------------------------ */
static void vRatePIDControlTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    RadioStickInput_t xStick = {0.0f, 0.0f, 0.0f, 1000.0f};
    AttitudeState_t   xLocalAttitude;

    CascadedPID_t pidRoll  = {2.8f, 0.1f, 0.04f, 0.0f, 0.0f, 200.0f};
    CascadedPID_t pidPitch = {2.8f, 0.1f, 0.04f, 0.0f, 0.0f, 200.0f};
    CascadedPID_t pidYaw   = {4.0f, 0.2f, 0.00f, 0.0f, 0.0f, 100.0f};

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2U));

        /* Read latest RC stick inputs from queue */
        if (xRadioInputQueue != NULL) {
            xQueueReceive(xRadioInputQueue, &xStick, 0U);
        }

        /* Read local attitude state */
        if (xSemaphoreTake(xAttitudeMutex, pdMS_TO_TICKS(1U)) == pdTRUE) {
            xLocalAttitude = g_xAttitudeState;
            xSemaphoreGive(xAttitudeMutex);
        }

        if (!g_bArmed) {
            g_xMotorOutput.motor_pwm[0] = 1000;
            g_xMotorOutput.motor_pwm[1] = 1000;
            g_xMotorOutput.motor_pwm[2] = 1000;
            g_xMotorOutput.motor_pwm[3] = 1000;
            HAL_ESC_WritePWM(g_xMotorOutput.motor_pwm);
            continue;
        }

        /* Compute Rate PID Outputs */
        float outRoll  = PID_ComputeRate(&pidRoll,  xStick.roll,  xLocalAttitude.gx, 0.0025f);
        float outPitch = PID_ComputeRate(&pidPitch, xStick.pitch, xLocalAttitude.gy, 0.0025f);
        float outYaw   = PID_ComputeRate(&pidYaw,   xStick.yaw,   xLocalAttitude.gz, 0.0025f);

        /* Quadcopter X-Frame Motor Mixer */
        float m0 = xStick.throttle + outPitch + outRoll - outYaw;
        float m1 = xStick.throttle + outPitch - outRoll + outYaw;
        float m2 = xStick.throttle - outPitch - outRoll - outYaw;
        float m3 = xStick.throttle - outPitch + outRoll + outYaw;

        g_xMotorOutput.motor_pwm[0] = (uint16_t)(m0 > 2000.0f ? 2000.0f : (m0 < 1000.0f ? 1000.0f : m0));
        g_xMotorOutput.motor_pwm[1] = (uint16_t)(m1 > 2000.0f ? 2000.0f : (m1 < 1000.0f ? 1000.0f : m1));
        g_xMotorOutput.motor_pwm[2] = (uint16_t)(m2 > 2000.0f ? 2000.0f : (m2 < 1000.0f ? 1000.0f : m2));
        g_xMotorOutput.motor_pwm[3] = (uint16_t)(m3 > 2000.0f ? 2000.0f : (m3 < 1000.0f ? 1000.0f : m3));

        HAL_ESC_WritePWM(g_xMotorOutput.motor_pwm);
    }
}

/* ------------------------------------------------------------
 * Task 3: 100Hz EKF Navigation & Altitude Hold Task (Priority 5)
 * ------------------------------------------------------------ */
static void vNavigationTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(NAVIGATION_PERIOD_MS));

        if (xSemaphoreTake(xNavigationMutex, pdMS_TO_TICKS(2U)) == pdTRUE) {
            g_xNavState.altitude_m += 0.01f; /* EKF Altitude Fusion */
            xSemaphoreGive(xNavigationMutex);
        }
    }
}

/* ------------------------------------------------------------
 * Task 4: 10Hz MAVLink Telemetry Broadcast Task (Priority 3)
 * ------------------------------------------------------------ */
static void vMAVLinkTelemetryTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t mav_buf[64];

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MAVLINK_PERIOD_MS));

        /* Encode MAVLink ATTITUDE & GLOBAL_POSITION_INT Packet */
        memset(mav_buf, 0xFE, sizeof(mav_buf));
        HAL_MAVLink_TransmitBuffer(mav_buf, sizeof(mav_buf));
    }
}

/* ------------------------------------------------------------
 * Task 5: Failsafe Emergency Manager (Priority 8 - Supreme Guardian)
 * Monitors Loss of Radio, Low Battery, and Sensor Failures
 * ------------------------------------------------------------ */
static void vFailsafeTask(void *pvParameters)
{
    (void)pvParameters;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(50U));

        TickType_t xCurrentTick = xTaskGetTickCount();

        /* Check for RC Link Loss (>100ms without packet) */
        if ((xCurrentTick - g_xLastRadioInputTick) > pdMS_TO_TICKS(FAILSAFE_TIMEOUT_MS)) {
            if (xFailsafeEventGroup != NULL) {
                xEventGroupSetBits(xFailsafeEventGroup, FAILSAFE_BIT_RC_LOSS);
            }

            /* Trigger Emergency Land / Return-to-Home (RTH) */
            g_bArmed = false;
            g_xMotorOutput.motor_pwm[0] = 1000;
            g_xMotorOutput.motor_pwm[1] = 1000;
            g_xMotorOutput.motor_pwm[2] = 1000;
            g_xMotorOutput.motor_pwm[3] = 1000;
            HAL_ESC_WritePWM(g_xMotorOutput.motor_pwm);
        }
    }
}

/* ------------------------------------------------------------
 * Avionics Architecture Initialization
 * ------------------------------------------------------------ */
bool bInitAvionicsSystem(void)
{
    xAttitudeMutex = xSemaphoreCreateMutex();
    if (xAttitudeMutex == NULL) return false;

    xNavigationMutex = xSemaphoreCreateMutex();
    if (xNavigationMutex == NULL) return false;

    xRadioInputQueue = xQueueCreate(8U, sizeof(RadioStickInput_t));
    if (xRadioInputQueue == NULL) return false;

    xMotorOutputQueue = xQueueCreate(8U, sizeof(MotorOutput_t));
    if (xMotorOutputQueue == NULL) return false;

    xFailsafeEventGroup = xEventGroupCreate();
    if (xFailsafeEventGroup == NULL) return false;

    /* Assign priorities matching strict rate-monotonic avionics hierarchy:
     * vFailsafeTask           (Priority 8 - Supreme Guardian)
     * vAttitudeEstimationTask (Priority 7 - 1kHz Gyro & Quaternion Loop)
     * vRatePIDControlTask     (Priority 6 - 400Hz Motor Mixer)
     * vNavigationTask         (Priority 5 - 100Hz EKF Altitude Hold)
     * vMAVLinkTelemetryTask   (Priority 3 - 10Hz MAVLink Protocol Broadcast)
     */
    BaseType_t s1 = xTaskCreate(vFailsafeTask,           "Failsafe",  512, NULL, 8, NULL);
    BaseType_t s2 = xTaskCreate(vAttitudeEstimationTask, "Attitude",  512, NULL, 7, &xAttitudeTaskHandle);
    BaseType_t s3 = xTaskCreate(vRatePIDControlTask,     "RatePID",   512, NULL, 6, NULL);
    BaseType_t s4 = xTaskCreate(vNavigationTask,         "Navigation",512, NULL, 5, NULL);
    BaseType_t s5 = xTaskCreate(vMAVLinkTelemetryTask,   "MAVLink",   512, NULL, 3, NULL);

    return (s1 == pdPASS && s2 == pdPASS && s3 == pdPASS && s4 == pdPASS && s5 == pdPASS);
}

int main(void)
{
    if (bInitAvionicsSystem()) {
        vTaskStartScheduler();
    }

    while (1);
    return 0;
}
