#ifndef __CONTROLER_H
#define __CONTROLER_H

#include "main.h"
#include "MPU6050.h"

extern volatile int work_mode; // 声明模式变量
extern MPU6050 MM;  // 声明 MPU6050 结构体变量，确保在 controler.c 中定义并初始化

void Joystick_Control(uint16_t *adc_value, float *angles);
void MPU6050_Control(float *angles);
void Servo_Movement(float *angles);
void Mode_Switch_Handler(uint16_t GPIO_Pin);

#endif
