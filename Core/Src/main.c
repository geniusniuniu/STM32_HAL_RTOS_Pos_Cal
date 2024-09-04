/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "position.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
/***************************************各类函数声明****************************************/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile Vector3 V3 = {0, 0, 0};
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  
	delay_init(168);
	//初始化不同的IIC接口
  	SoftSim_IIC_Init();
	VL53L0X_i2c_init();	
	
	//IMU初始化
	BMI088_InitFunc();
	MPU9250_DMP_InitFunc();
	
	//相关函数的参数初始化
	LPF2_ParamSet(50, 8);
	Pos_Filter_Init();
	HAL_TIM_Base_Start_IT(&htim2);	//开启TIM2中断
	exti_GPIO_Init();				//MPU9250外部中断引脚初始化
	printf("ALL Peripheral Initial OK\r\n");
	
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 
{
	static int exti_count = 0;
	if(GPIO_Pin == GPIO_PIN_2) 
	{
		exti_count++;
		mpu_mpl_get_data(&Pitch_9AX,&Roll_9AX,&Yaw_9AX);
		if(exti_count >= 10)
		{
			printf("9Axis:%.2f,%.2f,%.2f\r\n",Pitch_9AX,Roll_9AX,Yaw_9AX);
			exti_count = 0;
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	static int count_delay = 0;
	static int count = 0;
	static int count_prirnt = 0;
	
	#ifdef POSITION_CALC
		static float gyro_bias_z;
		static double gyro_ave_bias_z;
		static int count1;
		static char Yaw_turn = 0;
	#endif
	
	if(htim == &htim2)		//5ms定时中断
	{				
		//等待一段时间，让陀螺仪数据稳定
		if(count_delay < 1000)
		{
			count_delay++;
			return;
		}
		
		//读取加速度计数据
		bmi08a_get_data(&user_accel_bmi088, &dev);

		//对加速度计数据进行窗口滤波
		ACC_XYZ_Window_Filter(&user_accel_bmi088);

		V3.x = user_accel_bmi088.x*A;
		V3.y = user_accel_bmi088.y*A;
		V3.z = user_accel_bmi088.z*A;

		//读取陀螺仪数据
		bmi08g_get_data(&user_gyro_bmi088, &dev);
		gyro_x = user_gyro_bmi088.x*B;
		gyro_y = user_gyro_bmi088.y*B;
		gyro_z = user_gyro_bmi088.z*B;
				
		#ifdef POSITION_CALC
			gyro_z1 = (double)gyro_z * R2D;
			
			//对陀螺仪z轴数据进行窗口滤波
			gyro_z1 = Single_Window_Filter(gyro_z1);

			//对陀螺仪数据进行二阶低通滤波
			gyro_z1 = LPF2_Calculate(gyro_z1);
			
			//对陀螺仪z轴数据进行零偏消除
			if(fabs(gyro_z1) < THRESHOLD)
			{
				count1++;
				gyro_bias_z += gyro_z1;
				if(count1 > 5)
				{
					gyro_ave_bias_z = gyro_bias_z/5;
					count1 = 0;
					gyro_bias_z = 0;	
				}	
				//gyro_z1 -= gyro_ave_bias_z*0.835; 
			}
			gyro_z1 -= gyro_ave_bias_z*0.835; 
					
			//去除零偏之后积分出角度
			Yaw += (double)gyro_z1*0.005;
			//将角度限制在±180度之间
			if(Yaw > 180)
			{
				Yaw -= 360;
				Yaw_turn++;	//可以记录转过的圈数
			}
			else if(Yaw < -180)
			{
				Yaw += 360;
				Yaw_turn--;
			}
		#endif		
		count_prirnt++;
		count++;
		
		if(count_prirnt >= 20)	//100ms
		{
			printf("6Axis:%f,%f,%f\r\n",-Pitch,Roll,Yaw); 			
			count_prirnt = 0;
		}
			
		if(count == 100)		//500ms
		{
			LED0 = !LED0;
			count = 0;
		}
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	
  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
