#include "ESP8266.h"
#include <string.h>
#include <stdio.h>

extern uint8_t connection_established;

char rxBuffer[RX_BUFFER_SIZE];

int ESP8266_SendCommand(char* command, UART_HandleTypeDef *huart)
{
	// Enviar comando y retorno de carro
    HAL_UART_Transmit(huart, (uint8_t *)command, strlen(command), HAL_MAX_DELAY);
    HAL_UART_Transmit(huart, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);

    // Limpiar el búfer de recepción
    memset(rxBuffer, 0, RX_BUFFER_SIZE);
    // Recibir respuesta desde el UART
    HAL_UART_Receive(huart, (uint8_t *)rxBuffer, RX_BUFFER_SIZE - 1, RX_TIMEOUT);
    // Buscar OK en la respuesta y comprobar comando
    return ((strstr(rxBuffer, "OK") != NULL) && (strstr(rxBuffer, command) != NULL));
}
void ESP8266_InitAsAP(UART_HandleTypeDef *huart) //Password debe tener >=8 caracteres
{
    // Comprobar conexion
    while(!ESP8266_SendCommand("AT", huart));

    // Reiniciar el ESP8266
    while(!ESP8266_SendCommand("AT+RST", huart));

    // Configurar el ESP8266 en modo AP
    while(!ESP8266_SendCommand("AT+CWMODE=2", huart));

    // Establecer SSID y contraseña
    while(!ESP8266_SendCommand("AT+CWSAP=\""SSID"\",\""PASSWORD"\",1,3", huart));

    // Permitir múltiples conexiones
    while(!ESP8266_SendCommand("AT+CIPMUX=1", huart));

	// Crear servidor TCP en el puerto 8080
    while(!ESP8266_SendCommand("AT+CIPSERVER=1,8080", huart));

}
int ESP8266_CheckConnection(UART_HandleTypeDef *huart)
{
	// Enviar comando y retorno de carro
    HAL_UART_Transmit(huart, (uint8_t *)"AT+CIPSTATUS", strlen("AT+CIPSTATUS"), HAL_MAX_DELAY);
    HAL_UART_Transmit(huart, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
    // Limpiar el búfer de recepción
    memset(rxBuffer, 0, RX_BUFFER_SIZE);
    // Recibir respuesta desde el UART
    HAL_UART_Receive(huart, (uint8_t *)rxBuffer, RX_BUFFER_SIZE - 1, RX_TIMEOUT);
    // Buscar OK en la respuesta y comprobar conexion
    return ((strstr(rxBuffer, "OK") != NULL) && (strstr(rxBuffer, "CIPSTATUS:0") != NULL));
}
void ESP8266_SendByte(uint8_t data, UART_HandleTypeDef *huart)
{
	// Enviar 1 byte al enlace 0 (primera conexión TCP)
    HAL_UART_Transmit(huart, (uint8_t *)"AT+CIPSEND=0,1", strlen("AT+CIPSEND=0,1"), HAL_MAX_DELAY);
    HAL_UART_Transmit(huart, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
    HAL_Delay(1);
    HAL_UART_Transmit(huart, &data, 1, HAL_MAX_DELAY); // Enviar el byte
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{

	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{


	}
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
        __HAL_UART_CLEAR_OREFLAG(huart); // Limpia Overrun Error
        __HAL_UART_CLEAR_PEFLAG(huart); // Limpia Parity Error
        __HAL_UART_CLEAR_FEFLAG(huart); // Limpia Framing Error
        __HAL_UART_CLEAR_NEFLAG(huart); // Limpia Noise Error
        ESP8266_CheckConnection(huart);
	}
}
