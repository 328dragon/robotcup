#include "Lib_Math.h"

float Sqrt(float x)
{
    if (x <= 0.0f)
        return 0.0f;

    float xhalf = 0.5f * x;
    int i = *(int *)&x;
    i = 0x5f3759df - (i >> 1);
    x = *(float *)&i;
    x = x * (1.5f - xhalf * x * x);
    x = x * (1.5f - xhalf * x * x);
    return 1 / x;
}

float pow_f(float x, uint8_t n)
{
    float result = 1.0f;
    for (int i = 0; i < n; i++)
    {
        result *= x;
    }
    return result;
}

float LinearInterpolation(float x, float x1, float x2, float y1, float y2)
{
    return y1 + (y2 - y1) * (x - x1) / (x2 - x1);
}

float CubicSplineInterpolation(float x, float x1, float x2, float y1, float y2, float y1_, float y2_)
{
    float h = x2 - x1;
    float a = (y2 - y1) / h - h * (y2_ + 2 * y1_) / 6;
    float b = y1_ / 2;
    float c = (y2_ - y1_) / (6 * h);
    return a * pow_f(x - x1, 3) + b * pow_f(x - x1, 2) + c * pow_f(x - x1, 3);
}

float Polynomial3_Eval(const Polynomial3_t *poly, float x)
{
    return poly->a * x * x * x + poly->b * x * x + poly->c * x + poly->d;
}

float Polynomial3_d1(const Polynomial3_t *poly, float x)
{
    return 3 * poly->a * x * x + 2 * poly->b * x + poly->c;
}

float Polynomial3_d2(const Polynomial3_t *poly, float x)
{
    return 6 * poly->a * x + 2 * poly->b;
}

float Polynomial3_d3(const Polynomial3_t *poly, float x)
{
    return 6 * poly->a;
}

void CubicSpline_Init(CubicSpline_t *spline, Point p0, Point p1, Point slope)
{
    spline->x0 = p0.x;
    spline->y0 = p0.y;
    spline->x1 = p1.x;
    spline->y1 = p1.y;
    spline->m0 = slope.x;
    spline->m1 = slope.y;

    float h = spline->x1 - spline->x0;
    spline->a = spline->y0;
    spline->b = spline->m0;
    spline->c = (3 * (spline->y1 - spline->y0) / (h * h)) - (spline->m0 + 2 * spline->m1) / h;
    spline->d = (2 * (spline->y0 - spline->y1) / (h * h * h)) + (spline->m0 + spline->m1) / (h * h);
}

float CubicSpline_Eval(CubicSpline_t *spline, float x)
{
    if (x < spline->x0 || x > spline->x1)
        return 0;
    float h = x - spline->x0;
    return spline->a + spline->b * h + spline->c * h * h + spline->d * h * h * h;
}

float CubicSpline_dx(CubicSpline_t *spline, float x)
{
    if (x < spline->x0 || x > spline->x1)
        return 0;
    float h = x - spline->x0;
    return spline->b + 2 * spline->c * h + 3 * spline->d * h * h;
}
