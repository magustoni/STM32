#ifndef __RADIO_H
#define __RADIO_H

#include <main.h>

//Pin seleccion modo y LED azul (encendido a nivel bajo)
#define PIN_MODO GPIOC, GPIO_PIN_13 //RESET(0) -> automatico, LED ON | SET(1) -> manual, LED OFF

typedef struct
{
	//Handler temporizador
	TIM_HandleTypeDef *input;

	//Variables para entrada receptor
	float count; //Contador auxiliar para temporizador
	float duty; //Ciclo trabajo PWM (5% - 10%)

	//Modo de funcionamiento
	int mode;
} Radio;

HAL_StatusTypeDef radio_init(Radio* rc, TIM_HandleTypeDef* htim_input); //Inicializador de "objeto" radio
void radio_callback(Radio* rc); //Callback temporizador, llamar en HAL_TIM_IC_CaptureCallback SOLO si htim==rc.input y canal==1 (directo)
void radio_update(Radio* rc); //Actualizacion del modo funcionamiento, incluir en bucle principal

#endif
