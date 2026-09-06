#ifndef TASK_H
#define TASK_H

#include "FreeRTOS.h"

typedef void * TaskHandle_t;
typedef void (*TaskFunction_t)( void * );

BaseType_t xTaskCreate( TaskFunction_t pxTaskCode,
                        const char * const pcName,
                        const uint16_t usStackDepth,
                        void * const pvParameters,
                        UBaseType_t uxPriority,
                        TaskHandle_t * const pxCreatedTask );

void vTaskDelay( const TickType_t xTicksToDelay );
void vTaskDelayUntil( TickType_t * const pxPreviousWakeTime, const TickType_t xTimeIncrement );
TickType_t xTaskGetTickCount( void );
void vTaskStartScheduler( void );
void vTaskDelete( TaskHandle_t xTaskToDelete );

#endif /* TASK_H */
