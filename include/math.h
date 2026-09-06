#ifndef MATH_H
#define MATH_H

#include <stddef.h>

#define M_PI 3.14159265358979323846
#define HUGE_VAL (__builtin_huge_val())

#ifdef __cplusplus
extern "C" {
#endif

double fabs(double x);
float fabsf(float x);
double sqrt(double x);
float sqrtf(float x);
double sin(double x);
double cos(double x);
double tan(double x);
double atan2(double y, double x);
double pow(double x, double y);
double floor(double x);
double ceil(double x);

#ifdef __cplusplus
}
#endif

#endif /* MATH_H */
