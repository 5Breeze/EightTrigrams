#include "yaw_calculation.h"

// 定义常量
#define M_PI 3.1415926f
#define RAD_TO_DEG (180.0f / M_PI)

// 偏航角计算函数
float_t calculate_yaw(float_t acc_x, float_t acc_y, float_t acc_z, float_t mag_x, float_t mag_y, float_t mag_z) {
    // 计算重力方向的单位向量
    float_t norm_acc = sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
    float_t ax = acc_x / norm_acc;
    float_t ay = acc_y / norm_acc;
    float_t az = acc_z / norm_acc;

    // 计算磁力方向的单位向量
    float_t norm_mag = sqrt(mag_x * mag_x + mag_y * mag_y + mag_z * mag_z);
    float_t mx = mag_x / norm_mag;
    float_t my = mag_y / norm_mag;
    float_t mz = mag_z / norm_mag;

    // 计算俯仰角（Pitch）
    float_t pitch = asinf(-ax);

    // 计算滚转角（Roll）
    float_t roll = asinf(ay / cosf(pitch));

    // 计算偏航角（Yaw）
    float_t yaw = atan2f(
        mz * sinf(roll) - my * cosf(roll),
        mx * cosf(pitch) + my * sinf(pitch) * sinf(roll) + mz * sinf(pitch) * cosf(roll)
    );

    // 将弧度转换为角度
    yaw *= RAD_TO_DEG;

    // 调整偏航角范围为 [0, 360)
    if (yaw < 0.0f) {
        yaw += 360.0f;
    }

    return yaw;
}