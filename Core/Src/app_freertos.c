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
#include "usart.h"
#include "stdio.h"
#include "MPU6050.h"
#include "tim.h"
#include "string.h"
#include "gpio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */
#define MAX_MESSAGE_LEN 32
typedef struct {
  int message_len;
  char message_buffer[MAX_MESSAGE_LEN];
} printfMessage;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
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
//  if (direction == 0)
//    HAL_GPIO_WritePin(M2_pin1_GPIO_Port, M2_pin1_Pin, 1);
//  else
//    HAL_GPIO_WritePin(M2_pin1_GPIO_Port, M2_pin1_Pin, 0);
  HAL_GPIO_WritePin(M2_pin1_GPIO_Port, M2_pin1_Pin, !direction);
  for(x=0; x<steps; x=x+1)
  {
    HAL_GPIO_WritePin(M2_pin2_GPIO_Port, M2_pin2_Pin, 1);
    microDelay(delay);
    HAL_GPIO_WritePin(M2_pin2_GPIO_Port, M2_pin2_Pin, 0);
    microDelay(delay);
  }
}


#define UART_BUFFER_SIZE 256
unsigned char ucUartInputChar = '\0';
unsigned short int usBufferIndex = 0;
unsigned char ucUartInputBuffer[UART_BUFFER_SIZE];
unsigned short int usLastPrintedIndex = 0;
// *****************************************************    //
// Method name: HAL_UART_RxCpltCallback                     //
// Method description:  This method overrides the UART      //
//                      detection callback, which is        //
//                      triggered when a new char arrives   //
//                      It sends the char to the cyclic     //
//                      buffer and updates the buffer index //
// Input params:        huart                               //
//                      The specific uart that              //
//                      triggered the interrupt.            //
// Output params:       n/a                                 //
// *****************************************************    //

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart5){
    ucUartInputBuffer[usBufferIndex++] = ucUartInputChar;
    if(usBufferIndex > UART_BUFFER_SIZE - 1)
      usBufferIndex = 0;
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
/* Definitions for BT_uart */
osThreadId_t BT_uartHandle;
const osThreadAttr_t BT_uart_attributes = {
  .name = "BT_uart",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for printGateKeeper */
osThreadId_t printGateKeeperHandle;
const osThreadAttr_t printGateKeeper_attributes = {
  .name = "printGateKeeper",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for writeSetpoint */
osThreadId_t writeSetpointHandle;
const osThreadAttr_t writeSetpoint_attributes = {
  .name = "writeSetpoint",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for readFromHost */
osThreadId_t readFromHostHandle;
const osThreadAttr_t readFromHost_attributes = {
  .name = "readFromHost",
  .priority = (osPriority_t) osPriorityNormal6,
  .stack_size = 128 * 4
};
/* Definitions for readFromIMU */
osThreadId_t readFromIMUHandle;
const osThreadAttr_t readFromIMU_attributes = {
  .name = "readFromIMU",
  .priority = (osPriority_t) osPriorityRealtime1,
  .stack_size = 128 * 4
};
/* Definitions for updateControl */
osThreadId_t updateControlHandle;
const osThreadAttr_t updateControl_attributes = {
  .name = "updateControl",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for writeToHost */
osThreadId_t writeToHostHandle;
const osThreadAttr_t writeToHost_attributes = {
  .name = "writeToHost",
  .priority = (osPriority_t) osPriorityNormal5,
  .stack_size = 128 * 4
};
/* Definitions for convertSetpoint */
osThreadId_t convertSetpointHandle;
const osThreadAttr_t convertSetpoint_attributes = {
  .name = "convertSetpoint",
  .priority = (osPriority_t) osPriorityBelowNormal5,
  .stack_size = 128 * 4
};
/* Definitions for sendToHost */
osThreadId_t sendToHostHandle;
const osThreadAttr_t sendToHost_attributes = {
  .name = "sendToHost",
  .priority = (osPriority_t) osPriorityNormal5,
  .stack_size = 128 * 4
};
/* Definitions for moveYawMotor */
osThreadId_t moveYawMotorHandle;
const osThreadAttr_t moveYawMotor_attributes = {
  .name = "moveYawMotor",
  .priority = (osPriority_t) osPriorityBelowNormal2,
  .stack_size = 128 * 4
};
/* Definitions for moveRollMotor */
osThreadId_t moveRollMotorHandle;
const osThreadAttr_t moveRollMotor_attributes = {
  .name = "moveRollMotor",
  .priority = (osPriority_t) osPriorityBelowNormal1,
  .stack_size = 128 * 4
};
/* Definitions for printfQueue */
osMessageQueueId_t printfQueueHandle;
const osMessageQueueAttr_t printfQueue_attributes = {
  .name = "printfQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void defaultTaskFunc(void *argument);
void StartTask02(void *argument);
void BT_uart_func(void *argument);
void printGateKeeperFunc(void *argument);
void writeSetpointFunc(void *argument);
void readFromHostFunc(void *argument);
void readFromIMUFunc(void *argument);
void updateControlFunc(void *argument);
void writeToHostFunc(void *argument);
void convertSetpointToStepsFunc(void *argument);
void sendToHostFunc(void *argument);
void moveYawMotorFunc(void *argument);
void moveRollMotorFunc(void *argument);

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

  /* Create the queue(s) */
  /* creation of printfQueue */
  printfQueueHandle = osMessageQueueNew (128, sizeof(printfMessage), &printfQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(defaultTaskFunc, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* creation of BT_uart */
  BT_uartHandle = osThreadNew(BT_uart_func, NULL, &BT_uart_attributes);

  /* creation of printGateKeeper */
  printGateKeeperHandle = osThreadNew(printGateKeeperFunc, NULL, &printGateKeeper_attributes);

  /* creation of writeSetpoint */
  writeSetpointHandle = osThreadNew(writeSetpointFunc, NULL, &writeSetpoint_attributes);

  /* creation of readFromHost */
  readFromHostHandle = osThreadNew(readFromHostFunc, NULL, &readFromHost_attributes);

  /* creation of readFromIMU */
  readFromIMUHandle = osThreadNew(readFromIMUFunc, NULL, &readFromIMU_attributes);

  /* creation of updateControl */
  updateControlHandle = osThreadNew(updateControlFunc, NULL, &updateControl_attributes);

  /* creation of writeToHost */
  writeToHostHandle = osThreadNew(writeToHostFunc, NULL, &writeToHost_attributes);

  /* creation of convertSetpoint */
  convertSetpointHandle = osThreadNew(convertSetpointToStepsFunc, NULL, &convertSetpoint_attributes);

  /* creation of sendToHost */
  sendToHostHandle = osThreadNew(sendToHostFunc, NULL, &sendToHost_attributes);

  /* creation of moveYawMotor */
  moveYawMotorHandle = osThreadNew(moveYawMotorFunc, NULL, &moveYawMotor_attributes);

  /* creation of moveRollMotor */
  moveRollMotorHandle = osThreadNew(moveRollMotorFunc, NULL, &moveRollMotor_attributes);

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
//    printf("up....\r\n");
//    if(MPU6050_DataReady() == 0 && MPU6050_DataReady() == 1)
//    {
      MPU6050_ProcessData(&MPU6050);
      printf("acc: x=%d, y=%d, z=%d\r\n", MPU6050.acc_x_raw, MPU6050.acc_y_raw, MPU6050.acc_z_raw);
      printf("gyr: x=%d, y=%d, z=%d\r\n", (int) MPU6050.gyro_x, (int) MPU6050.gyro_y, (int) MPU6050.gyro_z);
//    }


    osDelay(100);
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
//    for(y=0; y<8; y=y+1) // 8 times
//    {
//      step(25, 0, 500); // 25 steps (45 degrees) CCV
//      HAL_Delay(500);
//    }
//    step(800, 0, 500); // 800 steps (4 revolutions ) CV
//    HAL_Delay(1000);
//    printf("turn....\r\n");
    osDelay(10000);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_BT_uart_func */
/**
* @brief Function implementing the BT_uart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BT_uart_func */
void BT_uart_func(void *argument)
{
  /* USER CODE BEGIN BT_uart_func */
//  char *char_to_print ="@";
//
//  /* Infinite loop */
//  HAL_UART_Transmit( &hlpuart1, "\r\nbluetooth IS on\r\n", 19, 100);
//  for(;;)
//  {
//    HAL_UART_Receive_IT(&huart5, &ucUartInputChar, 1);
//
//    // If buffer size differs from printed amount, print until they match
//    while ( usLastPrintedIndex != usBufferIndex ){
//
//      HAL_UART_Transmit( &hlpuart1, "bluetooth int: ", 15, 100);
//
////      char_to_print[0] = ucUartInputBuffer[usLastPrintedIndex++];
////      if (char_to_print[0] != '0' )
////        HAL_UART_Transmit( &hlpuart1, char_to_print, 1, 100);
////      else
////        HAL_UART_Transmit( &hlpuart1, "KEY_RELEASED", 12, 100);
////      HAL_UART_Transmit( &hlpuart1, "\r\n", 2, 100);
////      // if the printed size overflows, reset it
//      if ( ucUartInputBuffer[usLastPrintedIndex] == '0')
//        HAL_UART_Transmit( &hlpuart1, "KEY_RELEASED", 12, 100);
//      else
//        HAL_UART_Transmit( &hlpuart1, &( ucUartInputBuffer[usLastPrintedIndex] ), 1, 100);
//      usLastPrintedIndex++;
//
//
//      HAL_UART_Transmit( &hlpuart1, "\r\n", 2, 100);
//      if (usLastPrintedIndex >  UART_BUFFER_SIZE - 1)
//        usLastPrintedIndex = 0;
//    }
    osDelay(10000);
//  }
  /* USER CODE END BT_uart_func */
}

/* USER CODE BEGIN Header_printGateKeeperFunc */
/**
* @brief Function implementing the printGateKeeper thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_printGateKeeperFunc */
void printGateKeeperFunc(void *argument)
{
  /* USER CODE BEGIN printGateKeeperFunc */
  printfMessage xIncommingMessage;
  /* Infinite loop */
  for(;;)
  {

  if (osMessageQueueGet(printfQueueHandle, &xIncommingMessage, 0x0, 10) == osOK){
//    if(HAL_UART_Transmit(&huart3,(uint8_t *)xIncommingMessage.message_buffer, xIncommingMessage.message_len, 100) != HAL_OK)
    if(HAL_UART_Transmit(&hlpuart1,(uint8_t *)xIncommingMessage.message_buffer, xIncommingMessage.message_len, 100) != HAL_OK){
      Error_Handler();
      }
  }
    osDelay(100);
  }
  /* USER CODE END printGateKeeperFunc */
}

/* USER CODE BEGIN Header_writeSetpointFunc */
/**
* @brief Function implementing the writeSetpoint thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_writeSetpointFunc */
void writeSetpointFunc(void *argument)
{
  /* USER CODE BEGIN writeSetpointFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END writeSetpointFunc */
}

/* USER CODE BEGIN Header_readFromHostFunc */
/**
* @brief Function implementing the readFromHost thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_readFromHostFunc */
void readFromHostFunc(void *argument)
{
  /* USER CODE BEGIN readFromHostFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END readFromHostFunc */
}

/* USER CODE BEGIN Header_readFromIMUFunc */
/**
* @brief Function implementing the readFromIMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_readFromIMUFunc */
void readFromIMUFunc(void *argument)
{
  /* USER CODE BEGIN readFromIMUFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END readFromIMUFunc */
}

/* USER CODE BEGIN Header_updateControlFunc */
/**
* @brief Function implementing the updateControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_updateControlFunc */
void updateControlFunc(void *argument)
{
  /* USER CODE BEGIN updateControlFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END updateControlFunc */
}

/* USER CODE BEGIN Header_writeToHostFunc */
/**
* @brief Function implementing the writeToHost thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_writeToHostFunc */
void writeToHostFunc(void *argument)
{
  /* USER CODE BEGIN writeToHostFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END writeToHostFunc */
}

/* USER CODE BEGIN Header_convertSetpointToStepsFunc */
/**
* @brief Function implementing the convertSetpoint thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_convertSetpointToStepsFunc */
void convertSetpointToStepsFunc(void *argument)
{
  /* USER CODE BEGIN convertSetpointToStepsFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END convertSetpointToStepsFunc */
}

/* USER CODE BEGIN Header_sendToHostFunc */
/**
* @brief Function implementing the sendToHost thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sendToHostFunc */
void sendToHostFunc(void *argument)
{
  /* USER CODE BEGIN sendToHostFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END sendToHostFunc */
}

/* USER CODE BEGIN Header_moveYawMotorFunc */
/**
* @brief Function implementing the moveYawMotor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_moveYawMotorFunc */
void moveYawMotorFunc(void *argument)
{
  /* USER CODE BEGIN moveYawMotorFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END moveYawMotorFunc */
}

/* USER CODE BEGIN Header_moveRollMotorFunc */
/**
* @brief Function implementing the moveRollMotor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_moveRollMotorFunc */
void moveRollMotorFunc(void *argument)
{
  /* USER CODE BEGIN moveRollMotorFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END moveRollMotorFunc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
int _write(int file, char *ptr, int len)
{
  printfMessage xIncommingMessage;

  len = MIN(len, MAX_MESSAGE_LEN);
  xIncommingMessage.message_len = len;
  strncpy(xIncommingMessage.message_buffer, ptr, len);
  xIncommingMessage.message_buffer[len] = '\0';
  osMessageQueuePut(printfQueueHandle, &xIncommingMessage, 0x0, 100);
  return len;
}
/* USER CODE END Application */

