#include "main.h"
#include "usb_host.h"
#include "deneme.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

uint8_t rx_buffer_it = 0;
uint32_t Distance = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void Error_Handler(void);

void Servo_SetAngle(uint8_t servoIndex, float angle);
uint32_t Measure_Distance(void);


int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

  const char *testMsg = "System Initialized. Starting FreeRTOS Scheduler.\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t*)testMsg, strlen(testMsg), HAL_MAX_DELAY);

  Servo_SetAngle(6, 89.0f);
  Servo_SetAngle(3, 135.0f);
  Servo_SetAngle(5, 90.0f);
  Servo_SetAngle(2, 180.0f);
  Servo_SetAngle(1, 45.0f);

  xTaskCreate(
      ControlTask,
      "Control",
      256,
      NULL,
      tskIDLE_PRIORITY + 2,
      NULL);

  xTaskCreate(
      DistanceTask,
      "Distance",
      128,
      NULL,
      tskIDLE_PRIORITY + 1,
      NULL);

  xTaskCreate(
      LoggerTask,
      "Logger",
      128,
      NULL,
      tskIDLE_PRIORITY + 1,
      NULL);

  vTaskStartScheduler();

  while (1)
  {
      Error_Handler();
  }
}

void SystemClock_Config(void) { /* ... implementation ... */ }
static void MX_USART2_UART_Init(void) { /* ... implementation ... */ }
static void MX_USART3_UART_Init(void) { /* ... implementation ... */ }
static void MX_TIM2_Init(void) { /* ... implementation ... */ }

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_RCC_TIM4_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


static void MX_TIM3_Init(void)
{

}

static void MX_TIM4_Init(void)
{

}


void Servo_SetAngle(uint8_t servoIndex, float angle)
{
    if(angle < 0) angle = 0;
    if(angle > 180) angle = 180;
    uint16_t pulse = 500 + (uint32_t)((angle / 180.0f) * 2000.0f);

    switch(servoIndex)
    {
        case 3: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse); break;
        case 6: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse); break;
        case 5: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse); break;
        case 2: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse); break;
        case 1: __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse); break;
        default: break;
    }
}

uint32_t Measure_Distance(void)
{
    uint32_t duration = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    for (volatile int i=0; i<1000; i++);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

    while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET);
    uint32_t startTick = __HAL_TIM_GET_COUNTER(&htim2);
    while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET);
    uint32_t stopTick = __HAL_TIM_GET_COUNTER(&htim2);

    if(stopTick >= startTick)
        duration = stopTick - startTick;
    else
        duration = (0xFFFF - startTick) + stopTick;

    return duration / 58;
}

void Error_Handler(void) { /* ... implementation ... */ }
