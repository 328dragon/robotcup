#ifndef __LIB_MATH_H
#define __LIB_MATH_H

#include <stdint.h>

float Sqrt(float x);
float pow_f(float x, uint8_t n);

typedef struct
{
    double x;
    double y;
} Point;

typedef struct
{
    float a;
    float b;
    float c;
    float d;
} Polynomial3_t;

float Polynomial3_Eval(const Polynomial3_t *poly, float x);
float Polynomial3_d1(const Polynomial3_t *poly, float x);
float Polynomial3_d2(const Polynomial3_t *poly, float x);
float Polynomial3_d3(const Polynomial3_t *poly, float x);

typedef struct
{
    float x0, y0, x1, y1;
    float m0, m1;
    float a, b, c, d;
} CubicSpline_t;

void CubicSpline_Init(CubicSpline_t *spline, Point p0, Point p1, Point slope);
float CubicSpline_Eval(CubicSpline_t *spline, float x);
float CubicSpline_dx(CubicSpline_t *spline, float x);

float LinearInterpolation(float x, float x1, float x2, float y1, float y2);
float CubicSplineInterpolation(float x, float x1, float x2, float y1, float y2, float y1_, float y2_);

#endif
