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
std_msgs::Float32 encoder2;
ros::Publisher pubEncoder1("unav/enc1", &encoder1);
ros::Publisher pubEncoder2("unav/enc2", &encoder2);

void mot1_cb( const std_msgs::Int16& cmd_msg);
void mot2_cb( const std_msgs::Int16& cmd_msg);

ros::Subscriber<std_msgs::Int16> subMot1("unav/mot1", mot1_cb);
ros::Subscriber<std_msgs::Int16> subMot2("unav/mot2", mot2_cb);

char hello[] = "hello world!\0";
char serror[] = "error occurred\0";

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

osThreadId defaultTaskHandle;
osThreadId rosTaskHandle;
osThreadId motorsTaskHandle;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern "C" void StartDefaultTask(void const * argument);
extern "C" void StartRosTask(void const * argument);
extern "C" void StartMotorsTask(void const * argument);
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
  osThreadDef(motorsTask, StartMotorsTask, osPriorityAboveNormal, 0, 512);
  motorsTaskHandle = osThreadCreate(osThread(motorsTask), NULL);

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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void mot1_cb( const std_msgs::Int16& cmd_msg){
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, cmd_msg.data);
}

void mot2_cb( const std_msgs::Int16& cmd_msg){
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, cmd_msg.data);
}
     
extern "C" void StartRosTask(void const * argument){
    nh.initNode();
    TIM2->CNT = 100;
    nh.advertise(pubEncoder1);
    nh.advertise(pubEncoder2);
    int counter = 0;
    TickType_t c = xTaskGetTickCount();
    while (true) { 
      if(!counter--){
        counter = 100;
        nh.logdebug(hello);
      } 
      //pubLog.publish(&log_msg);
      nh.spinOnce();
      vTaskDelayUntil(&c, 10);  
    }
}

extern "C" void StartMotorsTask(void const * argument){
    float speed = 0;
    uint16_t lastreading1 = TIM2->CNT;
    uint16_t lastreading2 = TIM3->CNT;
    uint32_t lasttime = TIM5->CNT;
    float speed1 = 0;
    float speed2 = 0;

    TickType_t c = xTaskGetTickCount();
    while (true) { 
      uint32_t currenttime = TIM5->CNT;
      float dt =(float)(currenttime - lasttime) * 0.001; 
      lasttime = currenttime;
      uint32_t current1 = TIM2->CNT;
      uint32_t current2 = TIM3->CNT;

      int32_t deltaenc1 = (uint16_t)lastreading1 - current1; 
      int32_t deltaenc2 = (uint16_t)lastreading2 - current2;
      speed1 = speed1 * 0.005f + 0.995f * ((float)deltaenc1) / dt;
      speed2 = speed2 * 0.005f + 0.995f * ((float)deltaenc2) / dt;
      encoder1.data = speed1;
      encoder2.data = TIM14->CNT;
      pubEncoder1.publish(&encoder1);
      pubEncoder2.publish(&encoder2);
      //pubLog.publish(&log_msg);
      nh.spinOnce();
      vTaskDelayUntil(&c, 10);  
    }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
