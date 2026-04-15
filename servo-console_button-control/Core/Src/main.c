/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050 MM;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
	  MPU6050_Get_Angle(&MM);//(超低零点漂移)
	  //MPU6050_Get_Angle_Plus(&MM);//四元素法
  }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*/////////////////////硬件连接/////////////////////
OLED屏幕      SCL B10    SDA  B11
MPU6050       SCL B8     SDA  B9
USART(串口)	   TX  A9    RX  A10
ADC(摇杆)     VRX PB0    VRX  PB1  SW  
舵机          YAW(y)  PA0    PITCH(x) PA1
*//////////////////////硬件连接/////////////////////

// 重定向 fputc 函数
int fputc(int ch, FILE *f)
{
    // 使用 HAL 库发送一个字节的数据
    // huart1 是你定义的串口句柄，如果你用的是串口2，就改写成 &huart2
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    
    return ch;
}
// --- 舵机控制参数定义 ---
// 假设 TIM2 的计数频率为 1MHz (Prescaler=71, Clock=72MHz -> 1 tick = 1us)
// 如果不确定，请检查 CubeMX 中 TIM2 的 Prescaler 设置
#define SERVO_MIN_US  500   // 0度对应的脉宽 (0.5ms)
#define SERVO_MAX_US  2500  // 180度对应的脉宽 (2.5ms)
#define SERVO_MID_US  1500  // 90度对应的脉宽 (1.5ms)
#define SERVO_MAX_ANGLE 180.0f // 舵机最大角度

// 死区范围 (ADC值 0-4095, 中位2048)
#define JOYSTICK_DEADZONE 150
uint16_t adc_value[3]; // 存储 ADC 转换结果的数组
float angles[2]; // 存储计算出的角度
float actual_vdda = 3.3f; // 实际电压值，初始为默认值

float ADCToAngle(uint16_t adc_angle)
{
    return adc_angle * 180.0f / 4095.0f * (actual_vdda / 3.3f); // 用算出来的实际电压去计算角度
}

// 输入 MPU6050 的姿态角，输出角度
float MPUToAngle(float mpu_angle)
{
    // 补偿计算：以 90 度为中位进行反向补偿
    // 假设 MPU 水平时角度为 0，那么舵机应处于 90 度（中位）
    float target_angle = 90.0f - mpu_angle; 

    // 限幅保护：防止舵机撞到机械限位（建议保留 10-170 度的安全区间）
    if (target_angle < 10.0f) target_angle = 10.0f;
    if (target_angle > 170.0f) target_angle = 170.0f;

    return target_angle;
}

uint16_t AngleToCCR(float angle)
{
    // 限制角度范围
    if (angle < 0) angle = 0;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

    // 线性映射: CCR = Min + (Angle / MaxAngle) * (Max - Min)
    float pulse_us = SERVO_MIN_US + (angle / SERVO_MAX_ANGLE) * (SERVO_MAX_US - SERVO_MIN_US);
    
    // 假设 TIM2 计数频率为 1MHz (1us/tick)，直接返回 us 值
    // 如果你的 TIM2 频率不同，请在此处乘以系数。例如 2MHz 则除以 2。
    return (uint16_t)pulse_us;
}

void Joystick_Control(uint16_t *adc_value, float *angles)
{
    
    // 1. 保护：防止 DMA 数据未到位导致除以 0 (使用内部 1.2V 参考通道)
    // 假设 adc_value[2] 是内部参考电压通道
    if (adc_value[2] > 100) 
    {
        actual_vdda = 1.2f * 4095.0f / (float)adc_value[2];
    }
    else 
    {
        actual_vdda = 3.3f; // 默认值
    }

    // 2. 获取原始 ADC 值
    uint16_t raw_x = adc_value[0];
    uint16_t raw_y = adc_value[1];

    // 3. 死区处理 (防止中位抖动)
    if (raw_x > (2048 - JOYSTICK_DEADZONE) && raw_x < (2048 + JOYSTICK_DEADZONE)) raw_x = 2048;
    if (raw_y > (2048 - JOYSTICK_DEADZONE) && raw_y < (2048 + JOYSTICK_DEADZONE)) raw_y = 2048;

    // 4. 转换为角度 (0-180度)
    //0是yaw，1是pitch
    angles[0] = ADCToAngle(raw_y); // Yaw--y
    angles[1] = ADCToAngle(raw_x); // Pitch--x

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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1); // 校准 ADC
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_value, sizeof(adc_value) / sizeof(adc_value[0])); // 启动 ADC 转换并使用 DMA 存储结果
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Yaw (PA0)
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Pitch (PA1)
  // 初始化舵机到中位 (90度)
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, AngleToCCR(90.0f)); // Yaw--y
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, AngleToCCR(90.0f)); // Pitch--x

  MPU6050_init(&hi2c1);//初始化MPU6050(你使用的硬件iic)
  HAL_TIM_Base_Start_IT(&htim2);//定时器中断(确保每次读取的dt稳定)
  //printf("ID:%x",MPU6050_ID()); // 打印MPU6050的ID用于检车与设备连接是否正常
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Joystick_Control(adc_value, angles); // 读取摇杆并更新舵机角度
    //MPU6050_Control(angles); // 读取MPU6050的姿态角并更新舵机角度
    
    Servo_Movement(angles);

    HAL_Delay(50); // Delay for 1 second before the next reading
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
