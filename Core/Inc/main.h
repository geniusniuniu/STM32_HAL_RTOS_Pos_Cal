/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "sys.h"
#include "delay.h"

#include "gpio.h"

#include "stdio.h"
#include "math.h"

#include "bmi08x.h"
#include "bmi088.h"
#include "imu.h"

#include "myiic.h"
#include "mpu9250.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 

#include "vl53l0x.h"
#include "vl53l0x_platform.h"

#include "sensor.h"


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct 	
{
    float x;
    float y;
    float z;
} Vector3;


extern volatile Vector3 V3;
extern float Pitch_9AX,Roll_9AX,Yaw_9AX;	//ŷ����
extern short Temp_9AX;                     //�¶�

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

#define ORIGIN_A		0.000732421875								//ԭʼ���ٶȼ����� ��24/32768��
#define A 				0.007177734375f								//��ԭʼ���ٶȼ�����ת����m/s^2
//���ٶ�ԭʼ����ת����m/s^2
#define GRAVITY         9.81f
#define B 				0.00053263221801584764920766930190693f		//ԭʼ����������ת ��λ��rad/s��
#define R2D 			180.0f/M_PI_F								//����ת�Ƕ�

#define TOF_X_ADDR	0x56
#define TOF_Y_ADDR	0x5A	//VL53L0x�ĵ�ַ0x5b�����ã�����
#define TOF_Z_ADDR	0x5C


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
