#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <main.h>
#include <math.h>
#include <string.h>

//Constantes fisicas
#define PI 3.14159265359
#define REDUCCION 30.5 //Reducion en la transmision
#define RADIO 0.1 //Radio ruedas (m)
//Constantes PID
#define Kp 0.07722
#define Ki 0.1523
#define Kd 0

typedef struct
{
	//Handlers temporizadores
	TIM_HandleTypeDef *output; //Temporizador THR y STR
	TIM_HandleTypeDef *camera; //Temporizador CAM

	//Variables fisicas solicitadas
	float v_in, w_in; //Valores solicitados de v y w (m/s)
	float cam_in; //Orientacion solicitada de la camara

	//Variables PID
	float error;
	float error_i;
	float error_d;
	float timer;

	//Variables salida del control
	float d_motor; //Control ESC
	float d_direccion; //Control servo
	float d_camara; //Control camara

} Controller;

HAL_StatusTypeDef control_init(Controller*, TIM_HandleTypeDef*, TIM_HandleTypeDef*); //Inicializador de "objeto" control
void control_update(Controller*, float); //Actualizacion del control

#endif
