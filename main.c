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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

int mode = 1;
int param_mode = 0;
int P1 = 2;
int P2 = 2;
int P3 = 5;
double P4 = 1;
double P5 = 1;
uint8_t lcd_buffer[100];
uint8_t cls1[2] = {254,1}; // code to clear the lcd
uint16_t distance;
uint16_t time = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void lcd_print()
{
	HAL_UART_Transmit(&huart6,lcd_buffer,sizeof(lcd_buffer),1000);
	memset(lcd_buffer, 0, 100);
}

void Mode_check()
{
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET){
		if (mode != 1){
			HAL_UART_Transmit(&huart6,cls1,sizeof(cls1),1000);
			sprintf(lcd_buffer," Locked Mode \r \n");
			lcd_print();
			mode = 1;
		}
  }

	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET){
			if(mode != 2){
				HAL_UART_Transmit(&huart6,cls1,sizeof(cls1),1000);
				sprintf(lcd_buffer," Setup Mode \r \n");
				lcd_print();
				param_mode = 0;
				mode = 2;
			}
		}

	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET){
			if (mode != 3){
				HAL_UART_Transmit(&huart6,cls1,sizeof(cls1),1000);
				sprintf(lcd_buffer, " Run Mode \r \n");
				lcd_print();
				mode = 3;
			}
		}
}

void Parameter_check()			//check which parameter we want to set
{
	HAL_Delay(50);
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET){
		if(param_mode != 1 ){
			HAL_UART_Transmit(&huart6,cls1,sizeof(cls1),1000);
			sprintf(lcd_buffer," Setup P1: %d ft \r \n",P1);
			lcd_print();
			param_mode = 1;
		}
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET){
		if(param_mode != 2){
			HAL_UART_Transmit(&huart6,cls1,sizeof(cls1),1000);
			sprintf(lcd_buffer," Setup P2: %d ft \r \n",P2);
			lcd_print();
			param_mode = 2;
		}
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET){
		if (param_mode != 3){
			HAL_UART_Transmit(&huart6,cls1,sizeof(cls1),1000);
			sprintf(lcd_buffer," Setup P3: %d sec \r \n",P3);
			lcd_print();
			param_mode = 3;
		}
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET){
		if(param_mode != 4){
			HAL_UART_Transmit(&huart6,cls1,sizeof(cls1),1000);
			sprintf(lcd_buffer," Setup P4: %f speed ramp-up \r \n",P4);
			lcd_print();
			param_mode = 4;
		}
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET){
		if(param_mode != 5)
		{
			HAL_UART_Transmit(&huart6,cls1,sizeof(cls1),1000);
			sprintf(lcd_buffer," Setup P5: %f speed ramp-down \r \n",P5);
			lcd_print();
			param_mode = 5;
		}
	}
}

void change_Parameter_value()
{
	HAL_Delay(50);
	if(param_mode == 1){
		int change = 0;
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET && (P1 != 2)){
			change = 1;
			P1 = 2;
		}
		else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6 ) == GPIO_PIN_SET && (P1 != 4)){
			change = 1;
			P1 = 4;
		}
		else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_SET && (P1 != 6)){
			change = 1;
			P1 = 6;
		}
		else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_SET && (P1 != 8)){
			change = 1;
			P1 = 8;
		}
		if (change == 1)
		{
			HAL_UART_Transmit(&huart6,cls1,sizeof(cls1),1000);
			sprintf(lcd_buffer," Setup P1: %d ft \r \n",P1);
			lcd_print();
		}
	}
	else if(param_mode == 2){
		int change = 0;
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET && (P2 != 2)){
			change = 1;
			P2 = 2;
		}
		else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET && (P2 != 4)){
			P2 = 4;
			change = 1;
		}
		else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_SET && (P2 != 6)){
			P2 = 6;
			change = 1;
		}
		else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_SET && (P2 != 8)){
			P2 = 8;
			change = 1;
		}
		if(change == 1){
			HAL_UART_Transmit(&huart6,cls1,sizeof(cls1),1000);
			sprintf(lcd_buffer," Setup P2: %d ft \r \n",P2);
			lcd_print();
		}
	}
	else if(param_mode == 3){
		int change = 0;
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET && (P3 != 5)){
			P3 = 5;
			change = 1;
		}
		else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET && (P3 !=10)){
			P3 = 10;
			change = 1;
		}
		else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_SET && (P3 != 15)){
			P3 = 15;
			change = 1;
		}
		else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_SET && (P3 != 20)){
			P3 = 20;
			change = 1;
		}
		if(change == 1){
			HAL_UART_Transmit(&huart6,cls1,sizeof(cls1),1000);
			sprintf(lcd_buffer," Setup P3: %d sec \r \n",P3);
			lcd_print();
		}
	}
	else if(param_mode == 4){
		int change = 0;
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET && (P4 != 0.5)){
			P4 = 0.5;
			change = 1;
		}
		else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET && (P4 != 1)){
			P4 = 1;
			change = 1;
		}
		else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_SET && (P4 != 1.5)){
			P4 = 1.5;
			change = 1;
		}
		else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_SET && (P4 != 2)){
			P4 = 2;
			change = 1;
		}
		if(change == 1){
			HAL_UART_Transmit(&huart6,cls1,sizeof(cls1),1000);
			sprintf(lcd_buffer," Setup P4: %f speed ramp-up \r \n",P4);
			lcd_print();
		}
	}
	else if(param_mode == 5){
		int change = 0;
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET && (P5 != 0.5)){
			P5 = 0.5;
			change = 1;
		}
		else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET && (P5 != 1)){
			P5 = 1;
			change = 1;
		}
		else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_SET && (P5 != 1.5)){
			P5 = 1.5;
			change = 1;
		}
		else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_SET && (P5 != 2)){
			P5 = 2;
			change = 1;
		}
		if(change == 1){
			HAL_UART_Transmit(&huart6,cls1,sizeof(cls1),1000);
			sprintf(lcd_buffer," Setup P5: %f speed ramp-down \r \n",P5);
			lcd_print();
		}
	}
}

void Door_open()
{
	sprintf(lcd_buffer,"Opening Door \r \n"); //remove later
	lcd_print();
	int changed = 0;
	int cnt = 0;
	uint16_t pwm_val1  = 40;
	uint16_t pwm_val2  = 40;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10) == GPIO_PIN_SET){	}	//wait for motor to finish initializing - IDX

	while(pwm_val1 < 1000){ // ramping up motor
		HAL_Delay(50);

		TIM_OC_InitTypeDef sConfigOC;
		    sConfigOC.OCMode = TIM_OCMODE_PWM1;
		    sConfigOC.Pulse = pwm_val1;
		    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);
		    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

		pwm_val1 = pwm_val1 + (40*P4);
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10) == GPIO_PIN_SET){
			HAL_Delay(10);
			cnt = cnt + 1;
		}

		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);
		HAL_Delay(30);
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);			//Flash RED LED
	}

	while(1){ 			// count IDX revolutions

		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10) == GPIO_PIN_SET){
			HAL_Delay(10);
			cnt = cnt +1;
			sprintf(lcd_buffer,"revolution: %d \r \n",cnt);		//remove later
			lcd_print();
			changed = 0;
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);		//Flash RED LED
		}

		if(cnt == 95){ //pretty close
			HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3); 	//start ramping down
		}

		if(cnt == 120){
			pwm_val1 = 40;

			TIM_OC_InitTypeDef sConfigOC;
			    sConfigOC.OCMode = TIM_OCMODE_PWM1;
			    sConfigOC.Pulse = pwm_val1;
			    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
			    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
			    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);
			    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

			//TIM_OC_InitTypeDef sConfigOC;
			    sConfigOC.OCMode = TIM_OCMODE_PWM1;
			    sConfigOC.Pulse = pwm_val1;
			    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
			    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
			    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
			    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

			//Stop Motor
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
			break;
		}
		else if(cnt > 95){
			if(cnt == 115){ 	//close to stopping
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
			}
		}
	}

	sprintf(lcd_buffer,"Door is open");
	lcd_print();
	if(mode != 1){ 				//close door if not in locked mode
		HAL_Delay(P3*1000);		//remain open for P3 seconds
		Door_close(); 			//close door
	}
}

void Door_close()
{
	sprintf(lcd_buffer,"Closing Door \r \n"); //remove later
	lcd_print();
	int changed = 0;
	int cnt = 0;
	uint16_t pwm_val1  = 40;
	uint16_t pwm_value2  = 40;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);

	TIM_OC_InitTypeDef sConfigOC;
	    sConfigOC.OCMode = TIM_OCMODE_PWM1;
	    sConfigOC.Pulse = pwm_val1;
	    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
	    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

		while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10) == GPIO_PIN_SET){	}	//wait for motor to finish initializing - IDX

	while(pwm_val1 < 1000) // ramping up the motor.
	{
		HAL_Delay(50);

		TIM_OC_InitTypeDef sConfigOC;
		    sConfigOC.OCMode = TIM_OCMODE_PWM1;
		    sConfigOC.Pulse = pwm_val1;
		    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
		    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

		pwm_val1 = pwm_val1 + (40*P4);
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10) == GPIO_PIN_SET){
			HAL_Delay(10);
			cnt = cnt + 1;
		}
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);			//Flash RED LED
		HAL_Delay(30);
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);
	}

	while(1){	 //count IDX revolutions

		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10) == GPIO_PIN_SET){
			HAL_Delay(10);
			cnt = cnt+1;
			sprintf(lcd_buffer,"revolution: %d \r \n",cnt);
			lcd_print();
			changed = 0;
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);		//Flash RED LED
		}

		if( cnt == 95 ){ //pretty close
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); //start ramping down
		}


		if(cnt == 120){
			pwm_val1 = 40;

			TIM_OC_InitTypeDef sConfigOC;
			    sConfigOC.OCMode = TIM_OCMODE_PWM1;
			    sConfigOC.Pulse = pwm_val1;
			    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
			    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
			    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
			    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

			//TIM_OC_InitTypeDef sConfigOC;
			    sConfigOC.OCMode = TIM_OCMODE_PWM1;
			    sConfigOC.Pulse = pwm_val1;
			    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
			    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
			    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
			    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);


			//Stop Motor
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
			break;
		}
		else if(cnt > 95){
			if(cnt == 115){ //close to stopping
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
			}
		}
	}
	sprintf(lcd_buffer, "Door closed");
	lcd_print();
}

/*void run_Motor(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, SET);
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15) == GPIO_PIN_SET){
		//HAL_Delay(50);
		//if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7 ) == GPIO_PIN_RESET) //manually simulate detected object
		//{
			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, SET);		//RED LED - indicates door motor is in motion

			//Switch Motor on - Rotate clockwise to open door
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, SET);
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);		//GREEN LED - REMOVE!!!!!!!!

			int time_to_open_close = 10*1000;
			int hold_open_time = P3*1000;

			HAL_Delay(time_to_open_close);

			//Switch Motor off
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);		//GREEN LED - REMOVE!!!!!!!!
			HAL_Delay(hold_open_time);

			//Rotate motor counter-clockwise to close door
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);		//GREEN LED - REMOVE!!!!!!!!
			HAL_Delay(time_to_open_close);

			//Switch Motor off
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);		//GREEN LED - REMOVE!!!!!!!!
		//}
	}
}*/

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
		//uint8_t buff[] = "hello world \r \n";
		//uint8_t buff[] = "Switch closed\n";
		//uint8_t buff2[] = "Switch open\n";
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_Delay(500); // delay added for the lcd display to finish its setup.
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  //For distance proximity sensors
  //HAL_TIM_Base_Start_IT(&htim2);
  //HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3); //Start Output Compare on CH2 with interrupts enabled - PB10
  //HAL_UART_Receive_IT(&huart6, &byte_8, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  //sprintf(lcd_buffer, "%s mode\r\n", mode_str[mode-1]);
  //lcd_print();
  while(1){
	  while( mode == 1 )
	  	  {
	  		  Mode_check();
	  		  if( mode != 1)
	  		  {
	  			  break;
	  		  }

	  	  }
	  	  while( mode == 2 )
	  	  {

	  		  Mode_check();
	  		  if(mode != 2)
	  		  {
	  			  break;
	  		  }
	  		Parameter_check();
	  		change_Parameter_value();
	  	  }

	  	  while(mode == 3)
	  	  {
	  		  Mode_check();
	  		  if(mode != 3)
	  		  {
	  			  break;
	  		  }

	  		Door_open();
	  		Door_close();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xfff-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_3);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1600-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 50;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC3 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA6 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB12 PB13 PB14
                           PB4 PB5 PB6 PB7
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//Inside Distance Proximity Sensor
	/*if(GPIO_Pin == GPIO_PIN_4)
	{
		uint8_t buff[] = "wait before echo \r \n";
		uint8_t buff2[] = "got echo \r \n";
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) == GPIO_PIN_SET){
			HAL_UART_Transmit(&huart6, buff, sizeof(buff), 1000);
			time = __HAL_TIM_GetCounter(&htim2); 	//start timer
		}
		else{
			HAL_UART_Transmit(&huart6, buff2, sizeof(buff), 1000);
			time = __HAL_TIM_GetCounter(&htim2) - time;	//end timer

			distance = time/(148*12);
			HAL_UART_Transmit(&huart6,cls1,sizeof(cls1),1000);
			sprintf(lcd_buffer," Dist %f  \r \n",distance);
			lcd_print();

			if(distance <= P1)	//checking if the distance is within set parameters
			{
				//OPEN DOOR
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, SET); // Switch on the Output (LED) Pin PC0 - GREEN
				//Generate PWM signal for motor - PC8 & PC9
			}
		}
	}*/

	//Outside Distance Proximity Sensor
	/*if(GPIO_Pin == GPIO_PIN_1)
		{
			uint8_t buff[] = "wait before echo \r \n";
			uint8_t buff2[] = "got echo \r \n";
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1) == GPIO_PIN_SET){
				HAL_UART_Transmit(&huart6, buff, sizeof(buff), 1000);
				time = __HAL_TIM_GetCounter(&htim2); 	//start timer
			}
			else{
				HAL_UART_Transmit(&huart6, buff2, sizeof(buff), 1000);
				time = __HAL_TIM_GetCounter(&htim2) - time;	//end timer

				distance = time/(148*12);
				HAL_UART_Transmit(&huart6,cls1,sizeof(cls1),1000);
				sprintf(lcd_buffer," Dist %f  \r \n",distance);
				lcd_print();

				if(distance <= P2)	//checking if the distance is within set parameters
				{
					//OPEN DOOR
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, SET); // Switch on the Output (LED) Pin PC0 - GREEN
					//Generate PWM signal for motor - PC8 & PC9
				}
			}
		}*/

		//Collision Switch
		if(GPIO_Pin == GPIO_PIN_6)
		{
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
			HAL_Delay(30);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);			//Flash Yellow LED
		}

		//Position Limit Switch - Min
		/*if(GPIO_Pin == GPIO_PIN_12)
		{
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12) == GPIO_PIN_SET){
				P1 = 2;
				P2 = 2;
			}
			else
			{
				param
			}
		}*/

		//Position Limit Switch - Max
		/*if(GPIO_Pin == GPIO_PIN_11)
		{}*/
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
