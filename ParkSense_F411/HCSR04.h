#include "main.h"

#define DISTANCE_THRESHOLD 50 // in cm
#define HISTERESIS 5000 // in ms
#define LED_PIN GPIO_PIN_13
#define LED_PORT GPIOC
#define TRIG_PIN GPIO_PIN_6
#define TRIG_PORT GPIOA

#define TRUE 1
#define FALSE 0

void HCSR04_Init(TIM_HandleTypeDef *htim);
void HCSR04_Read(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
