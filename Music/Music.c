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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TEMPO 800
#define BLANCA TEMPO
#define NEGRA TEMPO/2
#define CORCHEA TEMPO/4
#define SEMI TEMPO/8
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){


}


//presc=1e5/frecuencia
void P(uint32_t delay){
	HAL_Delay(delay);
}
void S(uint32_t time){
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
	P(time);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
}
void SolLa2(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,963);
	P(time);
}
void LaSi2(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,858);
	P(time);
}
void Do3(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,765);
	P(time);
}
void ReMi3(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,643);
	P(time);
}
void Mi3(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,607);
	P(time);
}
void Fa3(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,573);
	P(time);
}
void Sol3(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,510);
	P(time);
}
void SolLa3(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,482);
	P(time);
}
void La3(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,455);
	P(time);
}
void LaSi3(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,429);
	P(time);
}
void Si3(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,405);
	P(time);
}
void Do4(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,382);
	P(time);
}
void Re4(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,341);
	P(time);
}
void ReMi4(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,321);
	P(time);
}
void Mi4(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,303);
	P(time);
}
void Fa4(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,286);
	P(time);
}
void FaSol4(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,270);
	P(time);
}
void Sol4(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,255);
	P(time);
}
void SolLa4(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,241);
	P(time);
}
void La4(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,227);
	P(time);
}
void LaSi4(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,215);
	P(time);
}
void Si4(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,202);
	P(time);
}
void Do5(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,191);
	P(time);
}
void DoRe5(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,180);
	P(time);
}
void Re5(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,170);
	P(time);
}
void ReMi5(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,161);
	P(time);
}
void Mi5(uint32_t time){
	__HAL_TIM_SET_PRESCALER(&htim1,151);
	P(time);
}

void Apple(){
	  Do4(SEMI);
	  S(CORCHEA);
	  Do4(SEMI);
	  S(SEMI);
	  Mi4(SEMI);
	  S(CORCHEA);
	  Mi4(SEMI);
	  S(SEMI);
	  La3(SEMI);
	  S(CORCHEA);
	  La3(SEMI);
	  S(SEMI);
	  Do4(SEMI);
	  S(CORCHEA);
	  Do4(SEMI);
	  S(SEMI);
	  Re4(SEMI);
	  S(CORCHEA);
	  Re4(SEMI);
	  S(SEMI);
	  Fa4(SEMI);
	  S(CORCHEA);
	  Fa4(SEMI);
	  S(SEMI);
	  Sol3(SEMI);
	  S(CORCHEA);
	  Sol3(SEMI);
	  S(SEMI);
	  Si3(SEMI);
	  S(CORCHEA);
	  Si3(SEMI);
	  S(SEMI);
}
void FurElise(){
	Mi5(CORCHEA);
	ReMi5(CORCHEA);
	Mi5(CORCHEA);
	ReMi5(CORCHEA);
	Mi5(CORCHEA);
	Si4(CORCHEA);
	Re5(CORCHEA);
	Do5(CORCHEA);
	La4(BLANCA);

	Do4(CORCHEA);
	Mi4(CORCHEA);
	La4(CORCHEA);
	Si4(BLANCA);

	Mi4(CORCHEA);
	SolLa4(CORCHEA);
	Si4(CORCHEA);
	Do5(BLANCA);

	//Rep 2
	Mi5(CORCHEA);
	ReMi5(CORCHEA);
	Mi5(CORCHEA);
	ReMi5(CORCHEA);
	Mi5(CORCHEA);
	Si4(CORCHEA);
	Re5(CORCHEA);
	Do5(CORCHEA);
	La4(BLANCA);

	Do4(CORCHEA);
	Mi4(CORCHEA);
	La4(CORCHEA);
	Si4(BLANCA);

	Mi4(CORCHEA);
	Do5(CORCHEA);
	Si4(CORCHEA);
	La4(BLANCA);
	S(NEGRA);
}
void Entertainer(){
	Re4(CORCHEA);
	ReMi4(CORCHEA);
	Mi4(CORCHEA);
	Do5(NEGRA);
	Mi4(CORCHEA);
	Do5(NEGRA);
	Mi4(CORCHEA);
	Do5(BLANCA);
	S(SEMI);

	Do5(CORCHEA);
	Re5(CORCHEA);
	ReMi5(CORCHEA);
	Mi5(CORCHEA);
	Do5(CORCHEA);
	Re5(CORCHEA);
	Mi5(NEGRA);
	Si4(CORCHEA);
	Re5(NEGRA);
	Do5(BLANCA);
	S(SEMI);

	//Rep 2
	Re4(CORCHEA);
	ReMi4(CORCHEA);
	Mi4(CORCHEA);
	Do5(NEGRA);
	Mi4(CORCHEA);
	Do5(NEGRA);
	Mi4(CORCHEA);
	Do5(BLANCA);
	S(SEMI);

	Si4(CORCHEA);
	La4(CORCHEA);
	FaSol4(CORCHEA);
	La4(CORCHEA);
	Do5(CORCHEA);
	Mi5(NEGRA);
	Re5(CORCHEA);
	Do5(CORCHEA);
	La4(CORCHEA);
	Re5(BLANCA);
	S(SEMI);

	//Rep 3
	Re4(CORCHEA);
	ReMi4(CORCHEA);
	Mi4(CORCHEA);
	Do5(NEGRA);
	Mi4(CORCHEA);
	Do5(NEGRA);
	Mi4(CORCHEA);
	Do5(BLANCA);
	S(SEMI);

	Do5(CORCHEA);
	Re5(CORCHEA);
	ReMi5(CORCHEA);
	Mi5(CORCHEA);
	Do5(CORCHEA);
	Re5(CORCHEA);
	Mi5(NEGRA);
	Si4(CORCHEA);
	Re5(NEGRA);
	Do5(BLANCA);
	S(SEMI);

	Do5(CORCHEA);
	Re5(CORCHEA);
	Mi5(CORCHEA);
	Do5(CORCHEA);
	Re5(CORCHEA);
	Mi5(NEGRA);

	Do5(CORCHEA);
	Re5(CORCHEA);
	Do5(CORCHEA);
	Mi5(CORCHEA);
	Do5(CORCHEA);
	Re5(CORCHEA);
	Mi5(NEGRA);

	Do5(CORCHEA);
	Re5(CORCHEA);
	Do5(CORCHEA);
	Mi5(CORCHEA);
	Do5(CORCHEA);
	Re5(CORCHEA);
	Mi5(NEGRA);

	Si4(CORCHEA);
	Re5(NEGRA);
	Do5(BLANCA);
	S(NEGRA);
}
void CanCan(){
	Re4(NEGRA);

	S(CORCHEA);
	Mi4(CORCHEA);
	Sol4(CORCHEA);
	FaSol4(CORCHEA);
	Mi4(CORCHEA);
	La4(CORCHEA);
	S(CORCHEA);
	La4(CORCHEA);
	S(CORCHEA);
	La4(CORCHEA);
	Si4(CORCHEA);
	FaSol4(CORCHEA);
	Sol4(CORCHEA);
	Mi4(CORCHEA);
	S(CORCHEA);
	Mi4(CORCHEA);
	S(CORCHEA);
	Mi4(CORCHEA);
	Sol4(CORCHEA);
	FaSol4(CORCHEA);
	Mi4(CORCHEA);
	Re4(CORCHEA);

	Re5(CORCHEA);
	DoRe5(CORCHEA);
	Si4(CORCHEA);
	La4(CORCHEA);
	Sol4(CORCHEA);
	FaSol4(CORCHEA);
	Mi4(CORCHEA);
	Re4(NEGRA);

	S(CORCHEA);
	Mi4(CORCHEA);
	Sol4(CORCHEA);
	FaSol4(CORCHEA);
	Mi4(CORCHEA);
	La4(CORCHEA);
	S(CORCHEA);
	La4(CORCHEA);
	S(CORCHEA);
	La4(CORCHEA);
	Si4(CORCHEA);
	FaSol4(CORCHEA);
	Sol4(CORCHEA);
	Mi4(CORCHEA);
	S(CORCHEA);
	Mi4(CORCHEA);
	S(CORCHEA);
	Mi4(CORCHEA);
	Sol4(CORCHEA);
	FaSol4(CORCHEA);
	Mi4(CORCHEA);
	Re4(CORCHEA);

	La4(CORCHEA);
	Mi4(CORCHEA);
	FaSol4(CORCHEA);
	Re4(CORCHEA);
	S(CORCHEA);
	Re4(CORCHEA);

	S(NEGRA);


}
void LetHerGo(){
	La4(CORCHEA);
	Si4(CORCHEA);
	La4(CORCHEA);
	Sol4(CORCHEA);
	Re4(CORCHEA);
	Mi4(CORCHEA);
	La4(NEGRA);
	La4(CORCHEA);
	Mi4(NEGRA);
	Si4(CORCHEA);
	Si4(BLANCA);
	S(CORCHEA);

	La4(CORCHEA);
	Si4(CORCHEA);
	La4(CORCHEA);
	Sol4(CORCHEA);
	Re4(CORCHEA);
	Mi4(CORCHEA);
	La4(NEGRA);
	La4(CORCHEA);
	Mi4(CORCHEA);
	Si4(NEGRA);
	La4(BLANCA);
	S(CORCHEA);

	La4(CORCHEA);
	Si4(CORCHEA);
	La4(CORCHEA);
	Sol4(CORCHEA);
	Re4(CORCHEA);
	Mi4(CORCHEA);
	La4(NEGRA);
	La4(CORCHEA);
	Mi4(NEGRA);
	La4(CORCHEA);
	Re5(CORCHEA);
	Si4(CORCHEA);
	Si4(NEGRA);
	Sol4(CORCHEA);
	La4(NEGRA);
	Si4(CORCHEA);
	Si4(BLANCA);
	S(CORCHEA);

}

void Fireflies(){
	LaSi2(SEMI);
	LaSi3(SEMI);
	Re5(SEMI);
	LaSi2(SEMI);
	LaSi3(SEMI);
	LaSi4(SEMI);
	LaSi3(SEMI);
	ReMi4(SEMI);

	ReMi3(SEMI);
	LaSi3(SEMI);
	Fa4(SEMI);
	ReMi4(SEMI);
	Fa4(SEMI);
	LaSi4(SEMI);
	Fa4(SEMI);
	ReMi4(SEMI);

	SolLa2(SEMI);
	ReMi3(SEMI);
	Do4(SEMI);
	LaSi3(SEMI);
	Do4(SEMI);
	ReMi4(SEMI);
	ReMi3(SEMI);
	SolLa2(SEMI);

	ReMi3(SEMI);
	SolLa3(SEMI);
	Do4(SEMI);
	LaSi3(SEMI);
	Do3(SEMI);
	LaSi3(SEMI);
	ReMi4(SEMI);
	Fa4(SEMI);
}
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,10);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Fireflies();
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 455;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 160;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 80;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

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
