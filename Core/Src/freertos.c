/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "retarget.h"
#include "IMUSO3.h"
#include "imu.h"
#include "IIC.h"
#include "math.h"
#include "control.h"
#include "mpu6050.h"
#include "bmp280.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

uint8_t flag_100Hz;
uint8_t flag_10Hz;
int temp;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern int sprintf(char *out, const char *format, ...);
/* USER CODE END Variables */
/* Definitions for put_msg */
osThreadId_t put_msgHandle;
const osThreadAttr_t put_msg_attributes = {
  .name = "put_msg",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for loop_100Hz */
osThreadId_t loop_100HzHandle;
const osThreadAttr_t loop_100Hz_attributes = {
  .name = "loop_100Hz",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for timer100Hz */
osTimerId_t timer100HzHandle;
const osTimerAttr_t timer100Hz_attributes = {
  .name = "timer100Hz"
};
/* Definitions for timer50Hz */
osTimerId_t timer50HzHandle;
const osTimerAttr_t timer50Hz_attributes = {
  .name = "timer50Hz"
};
/* Definitions for timer10Hz */
osTimerId_t timer10HzHandle;
const osTimerAttr_t timer10Hz_attributes = {
  .name = "timer10Hz"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void put_msg_fcn(void *argument);
void loop_100Hz_calc(void *argument);
void timer100Hz_cb(void *argument);
void timer50Hz_cb(void *argument);
void timer10Hz_cb(void *argument);

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

  /* Create the timer(s) */
  /* creation of timer100Hz */
  timer100HzHandle = osTimerNew(timer100Hz_cb, osTimerPeriodic, NULL, &timer100Hz_attributes);

  /* creation of timer50Hz */
  timer50HzHandle = osTimerNew(timer50Hz_cb, osTimerPeriodic, NULL, &timer50Hz_attributes);

  /* creation of timer10Hz */
  timer10HzHandle = osTimerNew(timer10Hz_cb, osTimerPeriodic, NULL, &timer10Hz_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  osTimerStart(timer100HzHandle,10);
  osTimerStart(timer50HzHandle,20);
  osTimerStart(timer10HzHandle,100);
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of put_msg */
  put_msgHandle = osThreadNew(put_msg_fcn, NULL, &put_msg_attributes);

  /* creation of loop_100Hz */
  loop_100HzHandle = osThreadNew(loop_100Hz_calc, NULL, &loop_100Hz_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_put_msg_fcn */
/**
  * @brief  Function implementing the put_msg thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_put_msg_fcn */
void put_msg_fcn(void *argument)
{
  /* USER CODE BEGIN put_msg_fcn */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END put_msg_fcn */
}

/* USER CODE BEGIN Header_loop_100Hz_calc */
/**
* @brief Function implementing the loop_100Hz thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_loop_100Hz_calc */
void loop_100Hz_calc(void *argument)
{
  /* USER CODE BEGIN loop_100Hz_calc */
  /* Infinite loop */
  uint8_t data_tmp[2];
  uint8_t count = 0;
  float temperature,pressure,humidity;
  for(;;)
  {
    if(flag_100Hz == 1){
      MPU6050_t data;
      flag_100Hz = 0;
      bmp280_read_float(&bmp280, &temperature, &pressure, &humidity);
//      MPU6050_Read_Accel(&data);
//      CtrlAttiRate();
//      IMUSO3Thread();
//      if(imu.accRaw[0
//      ]>10 || imu.accRaw[1] >10 || imu.accRaw[2]>10){
//        continue;
//      }
//      if(imu.accRaw[0]<-10 ||imu.accRaw[1] <-10 || imu.accRaw[2]<-10){
//        continue;
//      }
//      printf("acc1: %d\tacc2: %d\tacc3: %d\t\r\n",(int)imu.accRaw[0],(int)imu.accRaw[1],(int)imu.accRaw[2]);
//      printf("%d\n",(int)1.1f);
//      if(count>0){
        printf("pressure:%d\n",(int)pressure);
//      }
//      printf("pitch:%d\n",(int)imu.pitch*100);
    }if(flag_10Hz == 1){
//      printf("gyro1: %d\tgyro2: %d\tgyro3: %d\r\n",(int)imu.gyroRaw[0],(int)imu.gyroRaw[1],(int)imu.gyroRaw[2]);


//      printf("acc0: %d\t acc1: %d\t acc2: %d\r\n",imu.accADC[0],imu.accADC[1],imu.accADC[2]);
      flag_10Hz = 0;
    }
    osDelay(1);
  }
  /* USER CODE END loop_100Hz_calc */
}

/* timer100Hz_cb function */
void timer100Hz_cb(void *argument)
{
  /* USER CODE BEGIN timer100Hz_cb */
  flag_100Hz = 1;
  /* USER CODE END timer100Hz_cb */
}

/* timer50Hz_cb function */
void timer50Hz_cb(void *argument)
{
  /* USER CODE BEGIN timer50Hz_cb */
//  printf("Hello from 50Hz\n");
  /* USER CODE END timer50Hz_cb */
}

/* timer10Hz_cb function */
void timer10Hz_cb(void *argument)
{
  /* USER CODE BEGIN timer10Hz_cb */
  flag_10Hz = 1;
//  printf("Hello from 10Hz\n");
  /* USER CODE END timer10Hz_cb */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
