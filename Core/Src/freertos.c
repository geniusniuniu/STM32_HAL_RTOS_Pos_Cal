/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "semphr.h"
#include "position.h"
#include "event_groups.h"
#include "sensor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define BITMASK_TOF_X		(0x01)
#define BITMASK_TOF_Y		(0x02)
#define BITMASK_TOF_Z		(0x04)

#define BITMASK_TOF_OR		(BITMASK_TOF_X|BITMASK_TOF_Y|BITMASK_TOF_Z)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Task_TOF_X */
osThreadId_t Task_TOF_XHandle;
const osThreadAttr_t Task_TOF_X_attributes = {
  .name = "Task_TOF_X",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_TOF_Y */
osThreadId_t Task_TOF_YHandle;
const osThreadAttr_t Task_TOF_Y_attributes = {
  .name = "Task_TOF_Y",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_TOF_Z */
osThreadId_t Task_TOF_ZHandle;
const osThreadAttr_t Task_TOF_Z_attributes = {
  .name = "Task_TOF_Z",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_USART_Prin */
osThreadId_t Task_USART_PrinHandle;
const osThreadAttr_t Task_USART_Prin_attributes = {
  .name = "Task_USART_Prin",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Task_LED_Toggle */
osThreadId_t Task_LED_ToggleHandle;
const osThreadAttr_t Task_LED_Toggle_attributes = {
  .name = "Task_LED_Toggle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for I2C_Mutex */
osMutexId_t I2C_MutexHandle;
const osMutexAttr_t I2C_Mutex_attributes = {
  .name = "I2C_Mutex"
};
/* Definitions for Event_TOFsync */
osEventFlagsId_t Event_TOFsyncHandle;
const osEventFlagsAttr_t Event_TOFsync_attributes = {
  .name = "Event_TOFsync"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void taskTOF_X(void *argument);
void taskTOF_Y(void *argument);
void taskTOF_Z(void *argument);
void taskUSART_Print(void *argument);
void taskLED_Toggle(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of I2C_Mutex */
  I2C_MutexHandle = osMutexNew(&I2C_Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task_TOF_X */
  Task_TOF_XHandle = osThreadNew(taskTOF_X, NULL, &Task_TOF_X_attributes);

  /* creation of Task_TOF_Y */
  Task_TOF_YHandle = osThreadNew(taskTOF_Y, NULL, &Task_TOF_Y_attributes);

  /* creation of Task_TOF_Z */
  Task_TOF_ZHandle = osThreadNew(taskTOF_Z, NULL, &Task_TOF_Z_attributes);

  /* creation of Task_USART_Prin */
  Task_USART_PrinHandle = osThreadNew(taskUSART_Print, NULL, &Task_USART_Prin_attributes);

  /* creation of Task_LED_Toggle */
  Task_LED_ToggleHandle = osThreadNew(taskLED_Toggle, NULL, &Task_LED_Toggle_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of Event_TOFsync */
  Event_TOFsyncHandle = osEventFlagsNew(&Event_TOFsync_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_taskTOF_X */
/**
  * @brief  Function implementing the Task_TOF_X thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_taskTOF_X */
void taskTOF_X(void *argument)
{
  /* USER CODE BEGIN taskTOF_X */
	VL53L0X_Error status = 0;
	char buf[VL53L0X_MAX_STRING_LENGTH];//测试模式字符串字符缓冲区		
	/* Infinite loop */
	while(1)
	{			
		if(xSemaphoreTake(I2C_MutexHandle,pdMS_TO_TICKS(50)) == pdTRUE)	
		{			
			//Dis_Filter_Window(&Distance_data[Axis_X],Axis_X);
			status = vl53l0x_start_single_test(&vl53l0x_dev[Axis_X],Axis_X,&vl53l0x_data[Axis_X],buf);			
			if(status != VL53L0X_ERROR_NONE)
			{
				xEventGroupSetBits(Event_TOFsyncHandle,BITMASK_TOF_X);
				printf("Axis_X Status: %d\r\n",status);				
			}
			xSemaphoreGive(I2C_MutexHandle);
		}
		else// 超时的处理代码 
		{  
			printf("Axis_X Failed to obtain I2C BUS\r\n");  
		}  
		osDelay(pdMS_TO_TICKS(33));
	}
  /* USER CODE END taskTOF_X */
}

/* USER CODE BEGIN Header_taskTOF_Y */
/**
* @brief Function implementing the Task_TOF_Y thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskTOF_Y */
void taskTOF_Y(void *argument)
{
  /* USER CODE BEGIN taskTOF_Y */
	VL53L0X_Error status = 0;
	char buf[VL53L0X_MAX_STRING_LENGTH];//测试模式字符串字符缓冲区			
	/* Infinite loop */
	while(1)
	{	
		if(xSemaphoreTake(I2C_MutexHandle,pdMS_TO_TICKS(50)) == pdTRUE)	
		{	
			//Dis_Filter_Window(&Distance_data[Axis_Y],Axis_Y);
			status = vl53l0x_start_single_test(&vl53l0x_dev[Axis_Y],Axis_Y,&vl53l0x_data[Axis_Y],buf);
			
			if(status != VL53L0X_ERROR_NONE)
			{
				xEventGroupSetBits(Event_TOFsyncHandle,BITMASK_TOF_Y);
				printf("Axis_Y Status: %d\r\n",status);				
			}
			xSemaphoreGive(I2C_MutexHandle);
		}
		else// 超时的处理代码 
		{  
			printf("Axis_Y Failed to obtain I2C BUS\r\n");  
		}  
		osDelay(pdMS_TO_TICKS(33));
	}
  /* USER CODE END taskTOF_Y */
}

/* USER CODE BEGIN Header_taskTOF_Z */
/**
* @brief Function implementing the Task_TOF_Z thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskTOF_Z */
void taskTOF_Z(void *argument)
{
  /* USER CODE BEGIN taskTOF_Z */
	VL53L0X_Error status = 0;
	char buf[VL53L0X_MAX_STRING_LENGTH];//测试模式字符串字符缓冲区		
	
	/* Infinite loop */
	while(1)
	{	
		if(xSemaphoreTake(I2C_MutexHandle,pdMS_TO_TICKS(50)) == pdTRUE)	
		{			
			//Dis_Filter_Window(&Distance_data[Axis_Z],Axis_Z);
			status = vl53l0x_start_single_test(&vl53l0x_dev[Axis_Z],Axis_Z,&vl53l0x_data[Axis_Z],buf);
			
			if(status != VL53L0X_ERROR_NONE)
			{
				xEventGroupSetBits(Event_TOFsyncHandle,BITMASK_TOF_Z);
				printf("Axis_Z Status: %d\r\n",status);				
			}
			xSemaphoreGive(I2C_MutexHandle);			
		}
		else// 超时的处理代码 
		{  
			printf("Axis_Z Failed to obtain I2C BUS\r\n");  
		}  
		osDelay(pdMS_TO_TICKS(33));
	}
  /* USER CODE END taskTOF_Z */
}

/* USER CODE BEGIN Header_taskUSART_Print */
/**
* @brief Function implementing the Task_USART_Prin thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskUSART_Print */
void taskUSART_Print(void *argument)
{
  /* USER CODE BEGIN taskUSART_Print */
//	TaskStatus_t pxTaskInfo;
//	char pcWriteBuffer[256];
	/* Infinite loop */
//	vTaskList(pcWriteBuffer);
//	printf("/*******************KernalInfo******************/\r\n");
//	printf("%s\r\n",pcWriteBuffer);
//	printf("/*******************KernalInfo******************/\r\n");
	while(1)
	{			  		
//		vTaskGetInfo(Task_TOF_ZHandle, &pxTaskInfo, pdTRUE, eInvalid);
//		taskENTER_CRITICAL();//进入临界区	

//		printf("/*******************TaskGetInfo******************/\r\n");
//		printf("NumberOfTasks:%d\r\n",(int)uxTaskGetNumberOfTasks());
//		printf("TaskName:%s\r\n",pxTaskInfo.pcTaskName);
//		printf("State: %d\r\n",(int)pxTaskInfo.eCurrentState);
//		printf("Priority:%d\r\n",(int)pxTaskInfo.uxCurrentPriority);
//		printf("HighWaterMark:%d\r\n",pxTaskInfo.usStackHighWaterMark);
//		printf("/*******************TaskGetInfo******************/\r\n");		
//		taskEXIT_CRITICAL(); /* 退出临界区*/	
		
		taskENTER_CRITICAL();//进入临界区
			  	  
		Pos_Estimate(gyro_x, gyro_y, gyro_z, V3.x, V3.y, V3.z);	//位置估计 + BMI088角度解算  
		
		taskEXIT_CRITICAL(); /* 退出临界区*/	
		
		xEventGroupWaitBits(Event_TOFsyncHandle,BITMASK_TOF_OR,pdTRUE,pdTRUE,pdMS_TO_TICKS(50));
		Dis_Filter_Window(Distance_data);
		printf("Dis:%d,%d,%d\r\n",Distance_data[Axis_X],Distance_data[Axis_Y],Distance_data[Axis_Z]);
//		osDelay(100);
	} 
  /* USER CODE END taskUSART_Print */
}

/* USER CODE BEGIN Header_taskLED_Toggle */
/**
* @brief Function implementing the Task_LED_Toggle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskLED_Toggle */
void taskLED_Toggle(void *argument)
{
  /* USER CODE BEGIN taskLED_Toggle */
  /* Infinite loop */
  while(1)
  {
	LED0 =! LED0;
    osDelay(pdMS_TO_TICKS(500));
  }
  /* USER CODE END taskLED_Toggle */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

