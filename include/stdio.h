#ifndef _STDIO_H
#define _STDIO_H

#include "stddef.h"

int printf(const char *format, ...);
int sprintf(char *str, const char *format, ...);
int snprintf(char *str, size_t size, const char *format, ...);

#endif /* _STDIO_H */
