/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#include "stdio.h"
#include "MPU6050.h"
#include "tim.h"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
void microDelay (uint16_t delay)
{
  int a = 0;
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  do{
    a = __HAL_TIM_GET_COUNTER(&htim1);
  }while (a < delay);
}

void step (int steps, uint8_t direction, uint16_t delay)
{
  int x;
  if (direction == 0)
    HAL_GPIO_WritePin(M2_pin1_GPIO_Port, M1_pin1_Pin, 1);
  else
    HAL_GPIO_WritePin(M2_pin1_GPIO_Port, M1_pin1_Pin, 0);
  for(x=0; x<steps; x=x+1)
  {
    HAL_GPIO_WritePin(M2_pin2_GPIO_Port, M1_pin2_Pin, 1);
    microDelay(delay);
    HAL_GPIO_WritePin(M2_pin2_GPIO_Port, M1_pin2_Pin, 0);
    microDelay(delay);
  }
}



/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void defaultTaskFunc(void *argument);
void StartTask02(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  HAL_TIM_Base_Start(&htim1);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(defaultTaskFunc, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_defaultTaskFunc */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_defaultTaskFunc */
void defaultTaskFunc(void *argument)
{
  /* USER CODE BEGIN defaultTaskFunc */
  MPU6050_Initialization();
  /* Infinite loop */
  for(;;)
  {
    printf("up....\r\n");
    if(MPU6050_DataReady() == 1)
    {
      MPU6050_ProcessData(&MPU6050);
      //printf("%f, %f, %f\n", MPU6050.acc_x, MPU6050.acc_y, MPU6050.acc_z);
      //printf("%f, %f, %f\n", MPU6050.gyro_x, MPU6050.gyro_y, MPU6050.gyro_z);
//      printf("acc: x:%d, y:%d, z:%d\r\n", MPU6050.acc_x_raw, MPU6050.acc_y_raw, MPU6050.acc_z_raw);
      //printf("gyr: x:%d, y:%d, z:%d\r\n",  (int)MPU6050.gyro_x, (int)MPU6050.gyro_y, (int)MPU6050.gyro_z);
      printf("acc: x=%d, y=%d, z=%d\r\n", MPU6050.acc_x_raw, MPU6050.acc_y_raw, MPU6050.acc_z_raw);
      printf("gyr: x=%d, y=%d, z=%d\r\n", (int) MPU6050.gyro_x, (int) MPU6050.gyro_y, (int) MPU6050.gyro_z);
    }
    osDelay(1000);
  }
  /* USER CODE END defaultTaskFunc */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  int y;

  for(;;)
  {
    for(y=0; y<8; y=y+1) // 8 times
    {
      step(25, 0, 800); // 25 steps (45 degrees) CCV
      HAL_Delay(500);
    }
    step(800, 1, 5000); // 800 steps (4 revolutions ) CV
    HAL_Delay(1000);
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

