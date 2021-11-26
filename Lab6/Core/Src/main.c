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

#include <stdbool.h>
#include "math.h"
#include <stdio.h>               //needed library for sprintf
#include <string.h>             //needed library for strlen
#define SLAVE_ADDRESS 0x80     //PCA9685 I2C bus address   0b10000000
#define WAITTIME 1000

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

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

uint8_t TxBuffer[2];
uint8_t counter_for_menu=1;
uint8_t duty_cycle=50;
uint8_t sleep_bit=0;
uint8_t MODE0_reg = 0x00;
uint8_t PRE_SCALE_reg = 0xFE;
uint16_t frequency=300;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);   //turn on of the PB7 (output enable signal)

  while (1)
  {
	   	  uint8_t rcvBuf;
	  	  HAL_StatusTypeDef result;
	  	  result = HAL_UART_Receive(&huart3, &rcvBuf, 1, 1000);

	  	  if(counter_for_menu){
	  	  HAL_UART_Transmit(&huart3, (uint8_t *)"\033[2J", strlen("\033[2J"),10);  //Putty screen clear
	  	  HAL_UART_Transmit(&huart3, (uint8_t *)"\r\nMenu:\r\npress 0 for all_led_off\r\npress 1 for all_led_on\r\npress 2 for PWM_on"
	  			  "\r\npress 3 for duty cycle increase by 10\r\npress 4 for duty cycle decrease by 10\r\npress 5 for sleep mode on/off"
	  			  "\r\npress 6 for frequency increase by 100\r\npress 7 for frequency decrease by 100\r\n",
	  			  strlen("\r\nMenu:\r\npress 0 for all_led_off\r\npress 1 for all_led_on\r\npress 2 for PWM_on"
	  		  			  "\r\npress 3 for duty cycle increase by 10\r\npress 4 for duty cycle decrease by 10\r\npress 5 for sleep mode on/off"
	  		  			  "\r\npress 6 for frequency increase by 100\r\npress 7 for frequency decrease by 100\r\n"),1000);
	  	  counter_for_menu--;
	  	  }

  	  	  if (result == HAL_OK){

	  		  switch(rcvBuf){

	  			  case '0':
	  				 all_led_off(&hi2c1,TxBuffer);
	  				  HAL_UART_Transmit(&huart3, (uint8_t *)"ALL LED OFF\r\n", strlen("ALL LED OFF\r\n"),10);
	  				  break;

	  			  case '1':
	  				  all_led_on(&hi2c1,TxBuffer);
	  				  HAL_UART_Transmit(&huart3, (uint8_t *)"ALL LED ON\r\n", strlen("ALL LED ON\r\n"),10);
	  				  break;

	  			  case '2':
	  				  pwm_on(&hi2c1,TxBuffer);
	  				  HAL_UART_Transmit(&huart3, (uint8_t *)"PWM ON\r\n", strlen("PWM ON\r\n"),10);
	  				  break;

	  			  case '3':
	  				  duty_cycle = (duty_cycle==100) ? 100 : (duty_cycle + 10);
	  				  change_duty_cycle(duty_cycle,&hi2c1,TxBuffer);
					  HAL_UART_Transmit(&huart3, (uint8_t *)"duty cycle increase by 10\r\n", strlen("duty cycle increase by 10\r\n"),10);
					  break;

	  			  case '4':
	  				  duty_cycle=(duty_cycle==10) ? 10 : (duty_cycle-10);
	  				  change_duty_cycle(duty_cycle,&hi2c1,TxBuffer);
					  HAL_UART_Transmit(&huart3, (uint8_t *)"duty cycle decrease by 10\r\n", strlen("duty cycle decrease by 10\r\n"),10);
					  break;

	  			  case '5':
	  				  sleep_bit = (sleep_bit == 1) ? 0 : 1;
	  				  sleep_mode(&hi2c1, sleep_bit);
	  				  HAL_UART_Transmit(&huart3, (uint8_t *)"Sleep mode\r\n", strlen("Sleep mode\r\n"),10);
	  				  break;

	  			  case '6':
	  				 frequency=(frequency==1500) ? 1520 : (frequency+100);  //maximum PWM frequency is 1526 Hz
	  				 PCA9685_Frequency_Set(&hi2c1,frequency);
	  				 HAL_UART_Transmit(&huart3, (uint8_t *)"Frequency increase\r\n", strlen("Frequency increase\r\n"),10);
	  				 break;

	  			  case '7':
	  				 frequency=(frequency==100) ? 100 : (frequency-100);  //minimum PWM frequency is 24 Hz
	  				 PCA9685_Frequency_Set(&hi2c1,frequency);
	  				 HAL_UART_Transmit(&huart3, (uint8_t *)"Frequency decrease\r\n", strlen("Frequency decrease\r\n"),10);
	  				 break;

	  			  default:
	  				  HAL_UART_Transmit(&huart3, (uint8_t *)"Unexpected_command\r\n", strlen("Unexpected_command\r\n"),10);
	  				  break;
	  		  }
	  	  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
