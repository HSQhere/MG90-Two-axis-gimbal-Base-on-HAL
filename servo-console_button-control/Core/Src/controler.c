#include "controler.h"
#include "algorithm.h"
#include "tim.h"
#include <stdio.h>
#include "mpu6050.h"

// 死区范围 (ADC值 0-4095, 中位2048)
#define JOYSTICK_DEADZONE 150

void Joystick_Control(uint16_t *adc_value, float *angles) {
    // 1. 保护：防止 DMA 数据未到位导致除以 0 (使用内部 1.2V 参考通道)
    // 假设 adc_value[2] 是内部参考电压通道
    float actual_vdda = (adc_value[2] > 100) ? (1.2f * 4095.0f / adc_value[2]) : 3.3f;
    
    // 2. 获取原始 ADC 值
    uint16_t raw_x = adc_value[0];
    uint16_t raw_y = adc_value[1];

    // 3. 死区处理 (防止中位抖动)
    if (raw_x > (2048 - JOYSTICK_DEADZONE) && raw_x < (2048 + JOYSTICK_DEADZONE)) raw_x = 2048;
    if (raw_y > (2048 - JOYSTICK_DEADZONE) && raw_y < (2048 + JOYSTICK_DEADZONE)) raw_y = 2048;

    // 4. 转换为角度 (0-180度)
    //0是yaw，1是pitch
    angles[0] = ADCToAngle(raw_y, actual_vdda); // Yaw--y
    angles[1] = ADCToAngle(raw_x, actual_vdda); // Pitch--x

    printf("%u,%u,%.2f,%.2f\n", raw_x, raw_y, angles[0], angles[1]); // 输出原始 ADC 值和计算的角度，方便调试
}

void MPU6050_Control(float *angles)
{
    angles[0] = MPUToAngle(MM.yaw); // Yaw--y
    angles[1] = MPUToAngle(MM.pitch); // Pitch--x

    printf("%.3f,%.3f,%.3f\n",MM.roll,MM.pitch,MM.yaw);//欧拉角描述
}

void Servo_Movement(float *angles)
{
    // 映射为 PWM 占空比
    uint16_t ccr_yaw = AngleToCCR(angles[0]);
    uint16_t ccr_pitch = AngleToCCR(angles[1]);
    // 更新 TIM2 的 CCR 寄存器以控制舵机
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr_yaw);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr_pitch);
}


void Mode_Switch_Handler(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_7) {
        static uint32_t last_tick = 0;
        if (HAL_GetTick() - last_tick > 250) {
            // 这里建议加一个 ReadPin 复核，解决干扰自动触发问题
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET) {
                work_mode = !work_mode;
                last_tick = HAL_GetTick();
                // printf("Mode Switched: %d\n", work_mode); // 可选：增加串口输出确认模式切换
            }
        }
    }
}
