#include <controller.h>

//Inicializador de "objeto" c
HAL_StatusTypeDef control_init(Controller* c, TIM_HandleTypeDef* htim_output, TIM_HandleTypeDef* htim_camera)
{
	//Asignacion handlers
	c->output = htim_output; //Temporizador THR y STR
	c->camera = htim_camera; //Temporizador CAM

	c->d_motor = 1.5;
	c->d_direccion = 1.5;
	c->d_camara = 1.5;

	//Inicio salidas c PWM
	HAL_TIM_PWM_Start_IT(c->output, TIM_CHANNEL_2); //THR - PA9
	HAL_TIM_PWM_Start_IT(c->output, TIM_CHANNEL_3); //STR - PA10
	HAL_TIM_PWM_Start_IT(c->camera, TIM_CHANNEL_1); //CAM - PB8

	//Esperar al arranque de la ESC
	HAL_Delay(1000);

	c->error_i = 0;
	c->error_d = 0;
	c->timer = HAL_GetTick();

	return HAL_OK;
}

//Actualizacion del c
void control_update(Controller* c, float pid)
{
	//Accion control sobre ESC: v_in(m/s) -> pid -> d -> set_compare
	c->d_motor = 1.5 + pid;
	if(c->d_motor > 2) c->d_motor = 2;
	if(c->d_motor < 1) c->d_motor = 1;
	__HAL_TIM_SET_COMPARE(c->output, TIM_CHANNEL_2, 1000 * c->d_motor);

	//Accioncontrol sobre servo: w_in(rad/s) -> radio giro -> d -> set_compare
	c->d_direccion = 0.528 * c->w_in / c->v_in + 1.5;
	if(c->d_direccion > 2) c->d_direccion = 2;
	if(c->d_direccion < 1) c->d_direccion = 1;
	__HAL_TIM_SET_COMPARE(c->output, TIM_CHANNEL_3, 1000 * c->d_direccion); //Salida del control

	//Accion control sobre camara: cam_in(rad) -> d -> set_compare
	c->d_camara = 1.5 + 0.0107 * c->cam_in * 180 / PI;
	if(c->d_camara > 2.05) c->d_camara = 2.05;
	if(c->d_camara < 0.95) c->d_camara = 0.95;
	__HAL_TIM_SET_COMPARE(c->camera, TIM_CHANNEL_1, 1000 * c->d_camara); //Salida del control
}
