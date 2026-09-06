#ifndef SEMPHR_H
#define SEMPHR_H

#include "FreeRTOS.h"
#include "queue.h"

typedef QueueHandle_t SemaphoreHandle_t;

SemaphoreHandle_t xSemaphoreCreateMutex( void );
BaseType_t xSemaphoreTake( SemaphoreHandle_t xSemaphore, TickType_t xBlockTime );
BaseType_t xSemaphoreGive( SemaphoreHandle_t xSemaphore );

#endif /* SEMPHR_H */
