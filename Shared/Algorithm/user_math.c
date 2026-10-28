#include "user_math.h"

#include <math.h>

float Math_Clamp(float value, float min, float max)
{
    if (value < min)
    {
        return min;
    }
    if (value > max)
    {
        return max;
    }
    return value;
}

float Math_WrapDeg180(float deg)
{
    /* fmodf 的结果和被除数同号：先平移 180° 折到 0~360，再移回来。 */
    deg = fmodf(deg + 180.0f, 360.0f);
    if (deg < 0.0f)
    {
        deg += 360.0f;
    }
    return deg - 180.0f;
}
