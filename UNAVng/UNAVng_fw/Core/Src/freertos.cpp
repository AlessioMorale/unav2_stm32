/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usb_device.h"     
#include "usbd_cdc_if.h"
#include <ros.h>
#include <rosserial_msgs/Log.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
volatile uint8_t cdcbuffer[128] = "Test CDC\0";
volatile int32_t cdcSize = 9;

ros::NodeHandle nh;
rosserial_msgs::Log log_msg;

std_msgs::Float32 encoder1;
ros::Publisher pubEncoder1("unav/enc1", &encoder1);


char hello[] = "hello world!\0";
char serror[] = "error occurred";

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

osThreadId defaultTaskHandle;
osThreadId rosTaskHandle;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern "C" void StartDefaultTask(void const * argument);
extern "C" void StartRosTask(void const * argument);
extern "C" void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

void MX_USB_DEVICE_Init(void);
   
/* USER CODE END FunctionPrototypes */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
extern "C" void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
  osThreadDef(rosTask, StartRosTask, osPriorityAboveNormal, 0, 512);
  rosTaskHandle = osThreadCreate(osThread(rosTask), NULL);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
extern "C" void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN StartDefaultTask */
  uint8_t data[ 20 ];
  size_t receivedBytes = 0;
  const TickType_t blockTime = pdMS_TO_TICKS( 20 );

  uint16_t count = 0;
  uint16_t delta = 100;
  TickType_t c = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    count += delta;
    if(count > 0xFF00){
      delta *= -1;
    }
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, count);
    vTaskDelayUntil(&c, 1);    
  }
  /* USER CODE END StartDefaultTask */
}

extern "C" void StartRosTask(void const * argument){
    nh.initNode();
    nh.advertise(pubEncoder1);
    //nh.advertise(pubLog);
    //rosserial_msgs::Log l;
    //l.level= rosserial_msgs::Log::INFO;
    //l.msg =(char*) &hello;
    int counter = 0;
    while (true) { 
        if(!counter--){
          counter = 100;
          nh.logdebug(hello);
        } 
        encoder1.data++; 
        pubEncoder1.publish(&encoder1);
        //pubLog.publish(&log_msg);
        nh.spinOnce();
        vTaskDelay(1);
    }


}
/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
