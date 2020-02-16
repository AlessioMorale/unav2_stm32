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
#include "cmsis_os.h"
#include "main.h"
#include "timing.h"
#include <FreeRTOS.h>
#include <leds.h>
#include <leds_hal.h>
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <modules/motorcontrollermodule.h>
#include <modules/motormanagermodule.h>
#include <modules/rosnodemodule.h>
#include <modules/systemmodule.h>
#include <ros.h>
#include <rosserial_msgs/Log.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

osThreadId defaultTaskHandle;
unav::modules::RosNodeModule rosnode;
unav::modules::MotorManagerModule rosmotornode;
unav::modules::MotorControllerModule motorController;
unav::modules::SystemModule systemModule;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern "C" void StartDefaultTask(void const *argument);
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

  rosnode.initialize();
  rosmotornode.initialize();
  motorController.initialize();
  systemModule.initialize();
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
}

extern "C" void vApplicationIdleHook() {
  // Running led software pwm in idle task so there is a visible "alert" when
  // cpu is runnong put of free cpu cycles.
  leds_hal_updateLeds();
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
extern "C" void StartDefaultTask(void const *argument) {
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  leds_init();
  leds_setPattern(0, &leds_pattern_doublefast);
  leds_setPattern(1, &leds_pattern_off);
  leds_setPattern(2, &leds_pattern_off);

  /* USER CODE BEGIN StartDefaultTask */
  TickType_t c = xTaskGetTickCount();
  /* Infinite loop */
  for (;;) {
    leds_update();
    vTaskDelayUntil(&c, 50);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
