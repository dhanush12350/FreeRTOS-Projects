#ifndef PORTMACRO_H
#define PORTMACRO_H

#include <stdint.h>

typedef uint32_t TickType_t;
typedef int32_t BaseType_t;
typedef uint32_t UBaseType_t;

#define portMAX_DELAY ( TickType_t ) 0xffffffffUL
#define pdFALSE ( ( BaseType_t ) 0 )
#define pdTRUE  ( ( BaseType_t ) 1 )
#define pdPASS  ( ( BaseType_t ) 1 )
#define pdFAIL  ( ( BaseType_t ) 0 )

#define portTICK_PERIOD_MS ( ( TickType_t ) 1 )

#endif /* PORTMACRO_H */
