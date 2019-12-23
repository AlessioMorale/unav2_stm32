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
#include "tim.h"
#include "timing.h"
#include <FreeRTOS.h>
#include <leds.h>
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <modules/motorcontrollermodule.h>
#include <modules/motormanagermodule.h>
#include <modules/rosnodemodule.h>
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
extern "C" {
const led_pattern_t led_pattern_slowblink = {2, {{900, 0}, {100, 255}}};
const led_pattern_t led_pattern_fastblink = {2, {{450, 0}, {50, 255}}};
const led_pattern_t led_pattern_off = {1, {{1000, 0}}};
const led_pattern_t led_pattern_on = {1, {{1000, 255}}};
leds_status_t leds_status[NUM_LEDS];
volatile uint8_t ledvalue[NUM_LEDS] = {0};
}

osThreadId defaultTaskHandle;
unav::modules::RosNodeModule rosnode;
unav::modules::MotorManagerModule rosmotornode;
unav::modules::MotorControllerModule motorController;
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
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
}

extern "C" void vApplicationIdleHook() {
  const GPIO_PinState ledsOnState[] = LED_ARRAY_OF_STATES_ON;
  const GPIO_PinState ledsOffState[] = LED_ARRAY_OF_STATES_OFF;
  const GPIO_TypeDef *ledsGpio[] = LED_ARRAY_OF_GPIO;
  const uint16_t ledspin[] = LED_ARRAY_OF_PIN;
  static uint8_t count = 0;

  // Running led software pwm in idle task so there is a visible "alert" when
  // cpu is runnong put of free cpu cycles.
  count += UINT8_MAX / 20;
  for (int led = 0; led < NUM_LEDS; led++) {
    if (count < ledvalue[led]) {
      HAL_GPIO_WritePin((GPIO_TypeDef *)ledsGpio[led], ledspin[led],
                        ledsOnState[led]);
    } else {
      HAL_GPIO_WritePin((GPIO_TypeDef *)ledsGpio[led], ledspin[led],
                        ledsOffState[led]);
    }
  }
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
  // leds_status[0].pattern = &led_pattern_slowblink;

  /* USER CODE BEGIN StartDefaultTask */
  TickType_t c = xTaskGetTickCount();
  /* Infinite loop */
  for (;;) {
    uint32_t time = timing_getMs();
    for (int led = 0; led < NUM_LEDS; led++) {
      leds_status_t *status = &leds_status[led];
      if (status->pattern != nullptr &&
          status->intervalIndex < status->pattern->length) {
        if ((status->lastTime +
             status->pattern->intervals[status->intervalIndex].duration) <
            time) {
          status->lastTime = time;
          status->intervalIndex++;
          setLed(led,
                 status->pattern->intervals[status->intervalIndex].brightness);
        }
      } else {
        status->intervalIndex = 0;
      }
    }
    vTaskDelayUntil(&c, 10);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
