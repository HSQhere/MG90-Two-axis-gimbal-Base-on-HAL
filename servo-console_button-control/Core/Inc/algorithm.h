#ifndef __ALGORITHM_H
#define __ALGORITHM_H

#include "main.h"

float ADCToAngle(uint16_t adc_value, float actual_vdda);
float MPUToAngle(float mpu_angle);
uint16_t AngleToCCR(float angle);

#endif
