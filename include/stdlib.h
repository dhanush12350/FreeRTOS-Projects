#ifndef _STDLIB_H
#define _STDLIB_H

#include "stddef.h"

int atoi(const char *nptr);
long atol(const char *nptr);
void *malloc(size_t size);
void free(void *ptr);
int rand(void);
void srand(unsigned int seed);

#endif /* _STDLIB_H */
