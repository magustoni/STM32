#include <wt901c.h>

//Inicializador de "objeto" imu
HAL_StatusTypeDef wt901c_init(Wt901c* imu, UART_HandleTypeDef* huart)
{
	//Asignacion de handler UART
	imu->port = huart;

	//Iniciar alimentacion para resetear referencias
	HAL_GPIO_WritePin(PIN_IMU, GPIO_PIN_RESET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(PIN_IMU, GPIO_PIN_SET);

	//Primera recepcion
	return HAL_UART_Receive_IT(imu->port, imu->rx_buf, 11);
}

//Callback de recepcion UART, llamar en HAL_UART_RxCpltCallback
void wt901c_callback(Wt901c* imu)
{
	//Decodificar los 11 bytes de cada paquete
	for (int i = 0; i < 11; i++)
		wt901c_parse(imu, imu->rx_buf[i]);

	//Calcular nuevos valores de salida
	wt901c_calculate(imu);

	//Nueva recepcion
	HAL_UART_Receive_IT(imu->port, imu->rx_buf, 11);
}

//Decodificacion de los bytes entrantes
void wt901c_parse(Wt901c* imu, unsigned char byte)
{
	static unsigned char data_buffer[11];
	static unsigned char index = 0;

	data_buffer[index++] = byte;

	if (data_buffer[0] != 0x55) //Primer byte erroneo
	{
		index = 0;
		return;
	}

	if (index < 11) { return; } //Aun no se ha recibido el paquete completo

	else
	{
		switch (data_buffer[1]) //El segundo byte indica a que se corresponde el paquete de 11
		{
		case 0x50:	break; //Tiempo
		case 0x51:	memcpy(imu->raw_a, data_buffer + 2, 6); break; //Acelerometro
		case 0x52:	memcpy(imu->raw_w, data_buffer + 2, 6); break; //Giroscopio
		case 0x53:	memcpy(imu->raw_ang, data_buffer + 2, 6); break; //Angulos inclinacion
		case 0x54:	break; //Magnetometro (no utilizado)
		case 0x55:	break; //Status (no disponible en este modelo)
		case 0x56:	break; //Presion y altitud (no disponible en este modelo)
		case 0x57:	break; //GPS (no disponible en este modelo)
		case 0x58:	break; //GPSV (no disponible en este modelo)
		case 0x59:	memcpy(imu->raw_q, data_buffer + 2, 8); break; //Cuaternio orientacion
		}
		index = 0;
	}
}

//Calculo de los valores de salida a partir de los raw
void wt901c_calculate(Wt901c* imu)
{
	for (int i = 0; i < 3; i++) imu->a[i] = (float)imu->raw_a[i] / 32768 * 16 * G; //Aceleracion lineal en m/s2
	for (int i = 0; i < 3; i++) imu->w[i] = (float)imu->raw_w[i] / 32768 * 2000 * PI / 180; //Velocidad angular en rad/s
	for (int i = 0; i < 3; i++) imu->ang[i] = (float)imu->raw_ang[i] / 32768 * PI; //Angulo inclinacion en rad
	for (int i = 0; i < 4; i++) imu->q[i] = (float)imu->raw_q[i] / 32768; //Cuaternio de orientacion
}
