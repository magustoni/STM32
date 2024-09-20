#ifndef __WT901C_H
#define __WT901C_H

#include <main.h>
#include <string.h>

//Constantes fisicas
#define PI 3.14159265359
#define G 9.80665
//Pin alimentacion IMU
#define PIN_IMU GPIOB, GPIO_PIN_9 //RESET(0) -> off | SET(1) -> on

typedef struct{
	UART_HandleTypeDef* port; //Handler UART asignado
	unsigned char rx_buf[11]; //Buffer de recepcion UART

	short raw_a[3], raw_w[3], raw_ang[3], raw_q[4]; //Valores raw internos

	float a[3], w[3], ang[3], q[4]; //Valores de salida
} Wt901c;

HAL_StatusTypeDef wt901c_init(Wt901c* imu, UART_HandleTypeDef* huart); //Inicializador de "objeto" imu
void wt901c_callback(Wt901c* imu); //Callback de recepcion UART, llamar en HAL_UART_RxCpltCallback
void wt901c_parse(Wt901c* imu, unsigned char byte); //Decodificacion de los bytes entrantes
void wt901c_calculate(Wt901c* imu); //Calculo de los valores de salida a partir de los raw

#endif
