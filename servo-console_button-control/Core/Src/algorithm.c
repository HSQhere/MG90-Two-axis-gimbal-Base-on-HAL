#include "algorithm.h"
// --- 舵机控制参数定义 ---
// 假设 TIM2 的计数频率为 1MHz (Prescaler=71, Clock=72MHz -> 1 tick = 1us, ARR=20000-1 对应 20ms 周期)
// 如果不确定，请检查 CubeMX 中 TIM2 的 Prescaler 设置
#define SERVO_MIN_US  500   // 0度对应的脉宽 (0.5ms)
#define SERVO_MAX_US  2500  // 180度对应的脉宽 (2.5ms)
#define SERVO_MID_US  1500  // 90度对应的脉宽 (1.5ms)
#define SERVO_MAX_ANGLE 180.0f // 舵机最大角度

// 输入 ADC 值，输出角度
float ADCToAngle(uint16_t adc_value, float actual_vdda) {
    return adc_value * 180.0f / 4095.0f * (actual_vdda / 3.3f);
}

// 输入 MPU6050 的姿态角，输出角度
float MPUToAngle(float mpu_angle) {
    // 补偿计算：以 90 度为中位进行反向补偿
    // 假设 MPU 水平时角度为 0，那么舵机应处于 90 度（中位）
    float target_angle = 90.0f - mpu_angle; 

    // 限幅保护：防止舵机撞到机械限位（建议保留 10-170 度的安全区间）
    if (target_angle < 10.0f) target_angle = 10.0f;
    if (target_angle > 170.0f) target_angle = 170.0f;

    return target_angle;
}

// 输入角度，输出比较寄存器值
uint16_t AngleToCCR(float angle) {
    // 限制角度范围
    if (angle < 0) angle = 0;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

    // 线性映射: CCR = Min + (Angle / MaxAngle) * (Max - Min)
    float pulse_us = SERVO_MIN_US + (angle / SERVO_MAX_ANGLE) * (SERVO_MAX_US - SERVO_MIN_US);
    
    // 假设 TIM2 计数频率为 1MHz (1us/tick)，直接返回 us 值
    // 如果你的 TIM2 频率不同，请在此处乘以系数。例如 2MHz 则除以 2。
    return (uint16_t)pulse_us;
}
