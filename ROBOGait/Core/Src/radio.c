#include <radio.h>

//Inicializador de "objeto" radio
HAL_StatusTypeDef radio_init(Radio* rc, TIM_HandleTypeDef* htim_input)
{
	//Asignacion del handler
	rc->input = htim_input;

	rc->mode = 0;

	//Inicio medicion senales
	HAL_TIM_IC_Start_IT(rc->input, TIM_CHANNEL_1); //Canal directo radio - PA6
	HAL_TIM_IC_Start(rc->input, TIM_CHANNEL_2); //Canal indirecto radio

	return HAL_OK;
}

//Callback temporizador, llamar en HAL_TIM_IC_CaptureCallback SOLO si htim==rc->input y canal==1 (directo)
void radio_callback(Radio* rc)
{
	rc->count = HAL_TIM_ReadCapturedValue(rc->input, TIM_CHANNEL_1);
	if (rc->count > 0 && HAL_TIM_ReadCapturedValue(rc->input, TIM_CHANNEL_2) > 0)
		rc->duty = HAL_TIM_ReadCapturedValue(rc->input, TIM_CHANNEL_2) * 100 / rc->count;
}

//Actualizacion del modo funcionamiento, incluir en bucle principal
void radio_update(Radio* rc)
{
	//Seleccion modo
	if (rc->duty < 6) HAL_GPIO_WritePin(PIN_MODO, rc->mode = 0); //Boton A - Modo automatico
	//Valores intermedios: boton sin pulsar, el modo de funcionamiento se mantiene
	if (rc->duty > 8) HAL_GPIO_WritePin(PIN_MODO, rc->mode = 1); //Boton B - Modo manual
}
