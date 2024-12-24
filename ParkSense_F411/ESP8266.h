#include "main.h"

#define SSID "Parksense"
#define PASSWORD "PS123456"

#define RX_BUFFER_SIZE 256
#define RX_TIMEOUT 5000 //ms

#define TRUE 1
#define FALSE 0

int ESP8266_SendCommand(char *command, UART_HandleTypeDef *huart);
void ESP8266_InitAsAP(UART_HandleTypeDef *huart);
void ESP8266_SendByte(uint8_t data, UART_HandleTypeDef *huart);
int ESP8266_CheckConnection(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
