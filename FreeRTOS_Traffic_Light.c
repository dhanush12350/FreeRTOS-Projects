#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include <stdbool.h>
#include <stdint.h>

#define NS_GREEN_TIME_MS       10000U
#define NS_YELLOW_TIME_MS       3000U
#define EW_GREEN_TIME_MS       10000U
#define EW_YELLOW_TIME_MS       3000U
#define ALL_RED_SAFETY_MS       1000U
#define PEDESTRIAN_WALK_MS      6000U
#define EMERGENCY_GREEN_MS      8000U
#define FAILSAFE_BLINK_MS        500U

#define PIN_NS_RED    0U
#define PIN_NS_YELLOW 1U
#define PIN_NS_GREEN  2U
#define PIN_EW_RED    3U
#define PIN_EW_YELLOW 4U
#define PIN_EW_GREEN  5U
#define PIN_PED_WALK  6U
#define PIN_PED_STOP  7U

typedef enum {
    STATE_NS_GREEN_EW_RED = 0,
    STATE_NS_YELLOW_EW_RED,
    STATE_EW_GREEN_NS_RED,
    STATE_EW_YELLOW_NS_RED,
    STATE_PEDESTRIAN_CROSSING,
    STATE_EMERGENCY_OVERRIDE,
    STATE_FAILSAFE_BLINK
} TrafficState_t;

typedef enum {
    EMERGENCY_NONE = 0,
    EMERGENCY_NORTH_SOUTH,
    EMERGENCY_EAST_WEST
} EmergencyDir_t;

static SemaphoreHandle_t xPedestrianSemaphore = NULL;
static SemaphoreHandle_t xUARTMutex           = NULL;
static QueueHandle_t     xEmergencyQueue       = NULL;

static volatile bool           bPedestrianRequested = false;
static volatile TrafficState_t eCurrentState        = STATE_NS_GREEN_EW_RED;

static void HAL_GPIO_WritePin(uint8_t pin, bool bState)
{
    (void)pin;
    (void)bState;
}

static void System_Log(const char *pcMessage)
{
    if (xUARTMutex != NULL) {
        if (xSemaphoreTake(xUARTMutex, pdMS_TO_TICKS(10U)) == pdTRUE) {
            (void)pcMessage;
            xSemaphoreGive(xUARTMutex);
        }
    }
}

static void SetTrafficLights(bool nsR, bool nsY, bool nsG,
                             bool ewR, bool ewY, bool ewG,
                             bool pedWalk, bool pedStop)
{
    HAL_GPIO_WritePin(PIN_NS_RED,    nsR);
    HAL_GPIO_WritePin(PIN_NS_YELLOW, nsY);
    HAL_GPIO_WritePin(PIN_NS_GREEN,  nsG);

    HAL_GPIO_WritePin(PIN_EW_RED,    ewR);
    HAL_GPIO_WritePin(PIN_EW_YELLOW, ewY);
    HAL_GPIO_WritePin(PIN_EW_GREEN,  ewG);

    HAL_GPIO_WritePin(PIN_PED_WALK,  pedWalk);
    HAL_GPIO_WritePin(PIN_PED_STOP,  pedStop);
}

void EXTI0_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (xPedestrianSemaphore != NULL) {
        xSemaphoreGiveFromISR(xPedestrianSemaphore, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void vTrafficControlTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        switch (eCurrentState) {

        case STATE_NS_GREEN_EW_RED:
            System_Log("State: North-South GREEN | East-West RED");
            SetTrafficLights(false, false, true,   true, false, false,  false, true);
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(NS_GREEN_TIME_MS));
            eCurrentState = STATE_NS_YELLOW_EW_RED;
            break;

        case STATE_NS_YELLOW_EW_RED:
            System_Log("State: North-South YELLOW | East-West RED");
            SetTrafficLights(false, true, false,  true, false, false,  false, true);
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(NS_YELLOW_TIME_MS));

            SetTrafficLights(true, false, false,  true, false, false,  false, true);
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ALL_RED_SAFETY_MS));

            if (bPedestrianRequested) {
                eCurrentState = STATE_PEDESTRIAN_CROSSING;
            } else {
                eCurrentState = STATE_EW_GREEN_NS_RED;
            }
            break;

        case STATE_EW_GREEN_NS_RED:
            System_Log("State: East-West GREEN | North-South RED");
            SetTrafficLights(true, false, false,  false, false, true,  false, true);
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(EW_GREEN_TIME_MS));
            eCurrentState = STATE_EW_YELLOW_NS_RED;
            break;

        case STATE_EW_YELLOW_NS_RED:
            System_Log("State: East-West YELLOW | North-South RED");
            SetTrafficLights(true, false, false,  false, true, false,  false, true);
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(EW_YELLOW_TIME_MS));

            SetTrafficLights(true, false, false,  true, false, false,  false, true);
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ALL_RED_SAFETY_MS));

            if (bPedestrianRequested) {
                eCurrentState = STATE_PEDESTRIAN_CROSSING;
            } else {
                eCurrentState = STATE_NS_GREEN_EW_RED;
            }
            break;

        case STATE_PEDESTRIAN_CROSSING:
            System_Log("State: PEDESTRIAN CROSSING");
            SetTrafficLights(true, false, false,  true, false, false,  true, false);
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PEDESTRIAN_WALK_MS));

            bPedestrianRequested = false;
            eCurrentState = STATE_NS_GREEN_EW_RED;
            break;

        case STATE_EMERGENCY_OVERRIDE:
            vTaskDelay(pdMS_TO_TICKS(100U));
            break;

        case STATE_FAILSAFE_BLINK:
        default:
            System_Log("State: FAILSAFE BLINK");
            SetTrafficLights(false, true, false,  false, true, false,  false, true);
            vTaskDelay(pdMS_TO_TICKS(FAILSAFE_BLINK_MS));
            SetTrafficLights(false, false, false, false, false, false, false, true);
            vTaskDelay(pdMS_TO_TICKS(FAILSAFE_BLINK_MS));
            break;
        }
    }
}

static void vPedestrianTask(void *pvParameters)
{
    (void)pvParameters;

    while (1) {
        if (xSemaphoreTake(xPedestrianSemaphore, portMAX_DELAY) == pdTRUE) {
            System_Log("Event: Pedestrian Push-Button Pressed!");
            bPedestrianRequested = true;
        }
    }
}

static void vEmergencyTask(void *pvParameters)
{
    (void)pvParameters;
    EmergencyDir_t eDir;

    while (1) {
        if (xQueueReceive(xEmergencyQueue, &eDir, portMAX_DELAY) == pdTRUE) {
            System_Log("EVENT: EMERGENCY VEHICLE DETECTED!");
            
            TrafficState_t eSavedState = eCurrentState;
            eCurrentState = STATE_EMERGENCY_OVERRIDE;

            if (eDir == EMERGENCY_NORTH_SOUTH) {
                SetTrafficLights(false, false, true,  true, false, false,  false, true);
            } else {
                SetTrafficLights(true, false, false,  false, false, true,  false, true);
            }

            vTaskDelay(pdMS_TO_TICKS(EMERGENCY_GREEN_MS));

            SetTrafficLights(false, true, false,  false, true, false,  false, true);
            vTaskDelay(pdMS_TO_TICKS(2000U));

            eCurrentState = eSavedState;
        }
    }
}

bool bAppInitTrafficSystem(void)
{
    xPedestrianSemaphore = xSemaphoreCreateBinary();
    if (xPedestrianSemaphore == NULL) {
        return false;
    }

    xUARTMutex = xSemaphoreCreateMutex();
    if (xUARTMutex == NULL) {
        return false;
    }

    xEmergencyQueue = xQueueCreate(4U, sizeof(EmergencyDir_t));
    if (xEmergencyQueue == NULL) {
        return false;
    }

    BaseType_t s1 = xTaskCreate(vEmergencyTask,      "EmergencyTask", 512, NULL, 4, NULL);
    BaseType_t s2 = xTaskCreate(vTrafficControlTask, "TrafficCtrl",   512, NULL, 3, NULL);
    BaseType_t s3 = xTaskCreate(vPedestrianTask,     "PedestrianTask",512, NULL, 2, NULL);

    return (s1 == pdPASS && s2 == pdPASS && s3 == pdPASS);
}

int main(void)
{
    if (bAppInitTrafficSystem()) {
        vTaskStartScheduler();
    }

    while (1);
    return 0;
}
