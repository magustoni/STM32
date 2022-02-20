/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "i2c-lcd.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct{
	//Buffers para la transmision i2c
	uint8_t i2cTxBuf[2];
	uint8_t i2cRxBuf[2];

	//Aceleracion lineal y angular
	float ax, ay, az; //Valores entre -4g y +4g
	float wx, wy, wz; //Valores entre -500º/s y +500º/s

	//Variables para calculo de angulos
	float angX_acc, angY_acc; //Aporte de las aceleraciones (2%)
	float angX_rot, angY_rot; //Aporte de las rotaciones (98%)
	float t_aux, dt; //Para incrementos de tiempo

	//Variables de salida
	float anguloX, anguloY;
	float temp;
}MPU6050;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU_ADDR 0xD0 //Direccion del MPU
#define AX_ADDR 0x3B //Direccion del registro ax
#define AY_ADDR 0x3D //Direccion del registro ay
#define AZ_ADDR 0x3F //Direccion del registro az
#define WX_ADDR 0x43 //Direccion del registro wx
#define WY_ADDR 0x45 //Direccion del registro wy
#define WZ_ADDR 0x47 //Direccion del registro wz
#define T_ADDR 0x41 //Direccion del registro de temperatura
#define GCONFIG_ADDR 0x1B //Registro de configuracion giroscopo
#define ACONFIG_ADDR 0x1C //Registro de configuracion acelerometro
#define PWRCONFIG_ADDR 0x6B //Registro de configuracion de alimentacion
#define WHOAMI_ADDR 0x75 //Registro de test

#define LSB2G 8192.0  //Conversion LSB->G
#define LSB2DEGS 65.5 //Conversion LSB->º/s
#define RAD2DEG 180.0/3.1415926546 //Conversion rad->º
#define G 9.81 //Aceleracion de la gravedad

#define R1 9910.0 //Resistencia menor del divisor
#define R2 46840.0 //Resistencia mayor del divisor
#define VF 0.52 //Caida en el diodo
#define VMAX 2.96 //V maximo en pines ADC

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
MPU6050 mpu;
char buf[50];

float inclinacion_cal = 0.0; //Punto de calibracion
float inclinacion, inclinacion_max; //Valores en º
float aceleracion, aceleracion_max; //Valores en m/s2
float temperatura; //Valores en ºC
float voltaje; //Valores en V

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t);

float absol(float in);

void MPU_Init(); //Inicializacion del mpu
void MPU_Read(); //Lectura de los datos
void MPU_Calculate(); //Calculo de angulos
float V_Calculate(); //Voltaje medido
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  //Configuracion inicial MPU
  MPU_Init();
  //Configuracion inicial LCD
  lcd_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //Lectura de datos
	  	  MPU_Read();
	  //Calculos (filtro complementario)
	  	  MPU_Calculate();
	  //Valores de salida (solo positivos)
	  	  inclinacion = absol(mpu.anguloY - inclinacion_cal);
		  aceleracion = absol(mpu.ax) * G; //Se pasa el valor a m/s2
	  	  temperatura = mpu.temp;
	  	  voltaje = V_Calculate();
 	  //Comparacion con maximos
		  if(inclinacion > inclinacion_max) inclinacion_max = inclinacion;
		  if(aceleracion > aceleracion_max) aceleracion_max = aceleracion;

	  //Implementacion LCD
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)){ //Interruptor de seleccion
		//Valores actuales
		  lcd_clear();
		  lcd_put_cur(0,0);
		  if(aceleracion >= 10.0)
			  sprintf(buf, "Acc: %d    V: %d", (int)aceleracion, (int)voltaje);
		  else
			  sprintf(buf, "Acc: %d     V: %d", (int)aceleracion, (int)voltaje);
		  lcd_send_string(buf);

		  lcd_put_cur(1,0);
		  if(inclinacion >= 10.0)
			  sprintf(buf, "Inc: %d    T: %d", (int)inclinacion, (int)temperatura);
		  else
			  sprintf(buf, "Inc: %d     T: %d", (int)inclinacion, (int)temperatura);
		  lcd_send_string(buf);
	}
	else{
		//Valores maximos
		  lcd_clear();
		  lcd_put_cur(0,0);
		  sprintf(buf, "Acc_max: %d", (int)aceleracion_max);
		  lcd_send_string(buf);

		  lcd_put_cur(1,0);
		  sprintf(buf, "Inc_max: %d", (int)inclinacion_max);
		  lcd_send_string(buf);
	}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
//Callback de interrupcion usada para la calibracion
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN){
	if(GPIO_PIN == GPIO_PIN_0){
		inclinacion_cal = mpu.anguloY;
		inclinacion_max = 0.0;
		aceleracion_max = 0.0;
	}
}
//Valor absoluto con floats
float absol(float in){
	if(in >= 0) return in;
	else return -in;
}
//Inicializacion del MPU
void MPU_Init(){
	//Comprobacion inicial
	uint8_t check = 0;
	do HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, WHOAMI_ADDR, 1, &check, 1, HAL_MAX_DELAY);
	while(check != 0x68); //Registro de test devuelve 0x68 si todo esta bien

	//Configuraciones
	   mpu.i2cTxBuf[0]=0x00; //Wake up
	   	   HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, PWRCONFIG_ADDR, 1, mpu.i2cTxBuf, 1, HAL_MAX_DELAY);
	   mpu.i2cTxBuf[0]=0x08; //Rotacion max 500º/s
	 	  HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, GCONFIG_ADDR, 1, mpu.i2cTxBuf, 1, HAL_MAX_DELAY);
	   mpu.i2cTxBuf[0]=0x08; //Aceleracion max 4g
	  	  HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, ACONFIG_ADDR, 1, mpu.i2cTxBuf, 1, HAL_MAX_DELAY);
}
void MPU_Read(){
	//Aceleracion X
	  HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, AX_ADDR, 1, mpu.i2cRxBuf, 2, HAL_MAX_DELAY);
		  mpu.ax = (float)(((int16_t)(mpu.i2cRxBuf[0]<<8 | mpu.i2cRxBuf[1])) / LSB2G);
	//Aceleracion Y
	  HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, AY_ADDR, 1, mpu.i2cRxBuf, 2, HAL_MAX_DELAY);
	  	  mpu.ay = (float)(((int16_t)(mpu.i2cRxBuf[0]<<8 | mpu.i2cRxBuf[1])) / LSB2G);
	//Aceleracion Z
	  HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, AZ_ADDR, 1, mpu.i2cRxBuf, 2, HAL_MAX_DELAY);
	  	  mpu.az = (float)(((int16_t)(mpu.i2cRxBuf[0]<<8 | mpu.i2cRxBuf[1])) / LSB2G);
	//Rotacion X
	  HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, WX_ADDR, 1, mpu.i2cRxBuf, 2, HAL_MAX_DELAY);
	  	  mpu.wx = (float)(((int16_t)(mpu.i2cRxBuf[0]<<8 | mpu.i2cRxBuf[1])) / LSB2DEGS);
	//Rotacion Y
	  HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, WY_ADDR, 1, mpu.i2cRxBuf, 2, HAL_MAX_DELAY);
	  	  mpu.wy = (float)(((int16_t)(mpu.i2cRxBuf[0]<<8 | mpu.i2cRxBuf[1])) / LSB2DEGS);
	//Rotacion Z
	  HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, WZ_ADDR, 1, mpu.i2cRxBuf, 2, HAL_MAX_DELAY);
	  	  mpu.wz = (float)(((int16_t)(mpu.i2cRxBuf[0]<<8 | mpu.i2cRxBuf[1])) / LSB2DEGS);
	//Temperatura
	  HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, T_ADDR, 1, mpu.i2cRxBuf, 2, HAL_MAX_DELAY);
	  	  mpu.temp = (float)(((int16_t)(mpu.i2cRxBuf[0]<<8 | mpu.i2cRxBuf[1])) / 340.0 + 36.53);
}
void MPU_Calculate(){
	  //Aceleraciones
	  mpu.angX_acc = atan(-mpu.ay/sqrt(pow(mpu.ax,2) + pow(mpu.az,2)))*(RAD2DEG);
	  mpu.angY_acc = atan(mpu.ax/sqrt(pow(mpu.ay,2) + pow(mpu.az,2)))*(RAD2DEG);

	  //Rotaciones
	  mpu.dt = HAL_GetTick()-mpu.t_aux;
	  mpu.t_aux = HAL_GetTick();
	  mpu.angX_rot = mpu.wx*mpu.dt/1000;
	  mpu.angY_rot = mpu.wy*mpu.dt/1000;

	  //Resultante filtro
	  mpu.anguloX = 0.98*(mpu.anguloX+mpu.angX_rot) + 0.02*mpu.angX_acc;
	  mpu.anguloY = 0.98*(mpu.anguloY+mpu.angY_rot) + 0.02*mpu.angY_acc;
}
float V_Calculate(){
	static uint32_t v_raw; //ADC valor raw
	HAL_ADC_Start_DMA(&hadc1, &v_raw, 1);

	//Conversion del valor raw
	if(v_raw > 300) //Solo se calcula si se detecta alguna entrada
		return (v_raw * (1 + R2/R1) * VMAX / 4096.0) + VF;
	else //Sino se devuelve 0 para evitar mostrar el efecto del diodo
		return 0.0;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
