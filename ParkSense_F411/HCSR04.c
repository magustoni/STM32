#include "HCSR04.h"

extern float distance;
extern uint8_t read_flag, captured;

void HCSR04_Init(TIM_HandleTypeDef *htim)
{
	HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_1);
	__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
}
void HCSR04_Read(TIM_HandleTypeDef *htim)
{
	read_flag = FALSE;
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
	HAL_Delay(0.001);
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
	__HAL_TIM_ENABLE_IT(htim, TIM_IT_CC1);
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	static uint32_t time1 = 0, time2 = 0;

	if (htim->Instance == TIM2)
	{
		if (!captured)
		{
			time1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			captured = TRUE;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else
		{
			__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
			time2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			distance = (float)((time2 > time1) ? (time2 - time1) : (time2 + 4294967295 - time1));
			distance = distance * 0.0343 / 2.0;
			captured = FALSE;

			__HAL_TIM_SET_COUNTER(htim, 0);
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			read_flag = TRUE;
		}
	}
}
