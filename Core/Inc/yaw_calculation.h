#ifndef YAW_CALCULATION_H
#define YAW_CALCULATION_H

#include <stdint.h>
#include <math.h>

// 定义浮点类型
typedef float float_t;

// 偏航角计算函数
float_t calculate_yaw(float_t acc_x, float_t acc_y, float_t acc_z, float_t mag_x, float_t mag_y, float_t mag_z);

#endif // YAW_CALCULATION_H