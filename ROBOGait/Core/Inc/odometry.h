#ifndef __ODOMETRY_H
#define __ODOMETRY_H

//Constantes fisicas
#define PI 3.14159265359
#define REDUCCION 30.5 //Reducion en la transmision
#define RADIO 0.1 //Radio ruedas (m)

#define TIMER 20

#include <main.h>
#include <math.h>

typedef struct
{
	//Handlers
	TIM_HandleTypeDef* encoder; //Temporizador encoder
	ADC_HandleTypeDef* servos; //ADC realimentaciones

	//Variables fisicas actuales
	float px, py; //Valores actuales de x,y (m)
	float yaw;	//Orientacion actual (rad)
	float v, w; //Valores actuales de v y w (m/s y rad/s)
	float encoder_count, encoder_aux; //Cuenta del encoder transformada a (m)
	float v_aux[5];
	int index;

	float cam; //Orientacion actual de la camara (rad)
	uint32_t adc_buf[2]; //Buffer para conversion ADC

	//Temporizador
	float timer;

} Odometry;

HAL_StatusTypeDef odometry_init(Odometry* odom, TIM_HandleTypeDef* htim, ADC_HandleTypeDef* hadc); //Inicializador de "objeto" odometria
void odometry_callback(Odometry* odom); //Callback encoder, llamar en HAL_TIM_IC_CaptureCallback SOLO si htim==odom.encoder
void odometry_update(Odometry* odom, float yaw); //Actualizacion de la odometria, incluir en bucle principal

#endif
