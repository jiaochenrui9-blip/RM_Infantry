#ifndef USER_MATH_H
#define USER_MATH_H

/* 与硬件无关的小工具函数，底盘、云台都能用。 */

/* 把 value 限制在 [min, max]。 */
float Math_Clamp(float value, float min, float max);

/* 把角度换算到 -180~180 度。角度误差、跨 ±180° 的目标都要先过一遍这个。 */
float Math_WrapDeg180(float deg);

#endif /* USER_MATH_H */
