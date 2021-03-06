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
#include <string.h>   //needed library for strlen
#include <stdbool.h>
#include <string.h>
#define maxLines 20
#define lineMaxSize 100
#define waitTime 100

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
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart3;
/* USER CODE BEGIN PV */


uint8_t counter_for_menu=1;
uint8_t TransmitArray[100]={0};
char ReceiveArray[100]={0};
uint32_t address=0;
uint32_t memory = 0;
char timeCapsule[maxLines][lineMaxSize] = { };

const uint8_t readMemoryCode = 0x03;
const uint8_t getIdCode = 0x90;
const uint8_t writeEnableCode = 0x06;
const uint8_t writeDisableCode = 0x04;
const uint8_t eraseAllCode = 0x60;
const uint8_t enableWriteStatusCode = 0x50;
const uint8_t writeStatusCode = 0x01;
const uint8_t enableSOCode = 0x70;
const uint8_t disableSOCode = 0x80;
const uint8_t autoAddressIncrementCode = 0xAD;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void CS_change(){
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7)==GPIO_PIN_SET){
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_RESET);
	}
	else{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET);
	}
}

void flash_read(){

	for(uint8_t i=0;i<20;i++){
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET);  //CS = HIGH (CS is high means bus isn`t working)

		HAL_Delay(10);

		TransmitArray[0]=readMemoryCode;                 //Prepare READ-ID command
		TransmitArray[1]=address>>16;         //You should make these wierd adress shifts
		TransmitArray[2]=address>>8;
		TransmitArray[3]=address;

		if(address==77824){   //0+4096*19=77824 - address of line 20
			address=0;
		}
		else{
			  address+=4096;
		}

		CS_change(); //When CS is low transmit starts
		HAL_SPI_TransmitReceive(&hspi1,TransmitArray,(uint8_t *)ReceiveArray,100,1000);    //Exchange data

		CS_change();  //CS = HIGH
		HAL_Delay(10);
	    HAL_UART_Transmit(&huart3, (uint8_t *)"\r\n", strlen("\r\n"),10);               //send newline

	    for(uint8_t j=0;j<100;j++){       //filter off all the 0xFF
	    	if(ReceiveArray[j]==255){
	    		ReceiveArray[j]='\0';
	    	}
	    }
	    HAL_UART_Transmit(&huart3,(uint8_t *) ReceiveArray,sizeof ReceiveArray/sizeof ReceiveArray[0],0XFFFF);
	}
}

void pin_control(bool state){
	if (state == true)
	{
		if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7) == GPIO_PIN_SET)
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
	}
	else
	{
		if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7) == GPIO_PIN_RESET)
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
	}
}

void setWriteStatus(){
	uint8_t status[] = {writeStatusCode, 0};

	pin_control(true);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&enableWriteStatusCode, 1, waitTime);
	pin_control(false);

	pin_control(true);
	HAL_SPI_Transmit(&hspi1, status, 2, waitTime);
	pin_control(false);
}

void setHEOW(bool state){
	pin_control(true);
	HAL_SPI_Transmit(&hspi1, state == true ? (uint8_t *)&enableSOCode : (uint8_t *)&disableSOCode, 1, waitTime);
	pin_control(false);
}

void waitForEndOfWriting(){
	pin_control(true);

	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET);

	pin_control(false);
}

void setWriteMode(bool state){
	pin_control(true);
	HAL_SPI_Transmit(&hspi1, state == true ? (uint8_t *)&writeEnableCode : (uint8_t *)&writeDisableCode, 1, waitTime);
	pin_control(false);
}

void flash_clear(){
	    setWriteMode(true);
		pin_control(true);
		HAL_SPI_Transmit(&hspi1, (uint8_t *)&eraseAllCode, 1, waitTime);
		pin_control(false);
	    setWriteMode(false);
	    HAL_Delay(100);
}

void flash_write(uint32_t memory, uint8_t *line, uint8_t lineLength){
	setWriteStatus();
	setWriteMode(true);
	setHEOW(true);

	uint8_t startWritingPoint[] = {autoAddressIncrementCode, memory >> 16, memory >> 8, memory, line[0], line[1]};
	pin_control(true);
	HAL_SPI_Transmit(&hspi1, startWritingPoint, sizeof(startWritingPoint), waitTime);
	pin_control(false);
	waitForEndOfWriting();

	uint8_t twoBytesToPass[] = {autoAddressIncrementCode, line[0], line[1]};
	for (int i = 2; i < lineLength; i += 2){
		twoBytesToPass[1] = line[i];
		if (i + 1 < lineLength)
			twoBytesToPass[2] = line[i + 1];
		else
			twoBytesToPass[2] = 255;
		pin_control(true);
		HAL_SPI_Transmit(&hspi1, twoBytesToPass, sizeof(twoBytesToPass), waitTime);
		pin_control(false);

		waitForEndOfWriting();
	}

	setWriteMode(false);
	setHEOW(false);
}

void flash_write_text(){

	strcat(timeCapsule[0], "From: Hrona Yurii, yura.grona.011i@gmail.com\r\n");
	strcat(timeCapsule[1], "Mentor: Vitalii Kostiuk, vitalii.kostiuk@globallogic.com\r\n");
	strcat(timeCapsule[2], "Date: 30.11.2021\r\n");
	strcat(timeCapsule[3], "TIME CAPSULE\r\n");
	strcat(timeCapsule[4], "Les Go??mons (Khrystyna Soloviy)\r\n");
	strcat(timeCapsule[5], "Dessous la vague bougent\r\n");
	strcat(timeCapsule[6], "Les go??mons\r\n");
	strcat(timeCapsule[7], "Mes amours leur ressemblent\r\n");
	strcat(timeCapsule[8], "Il n'en reste il me semble\r\n");
	strcat(timeCapsule[9], "Que go??mons\r\n");
	strcat(timeCapsule[10], "Que des fleurs arrach??es\r\n");
	strcat(timeCapsule[11], "Se mourant comme les\r\n");
	strcat(timeCapsule[12], "Noirs go??mons\r\n");

	memory = 0;
	for (int i = 0; i < maxLines; i++){
		  flash_write(memory, (uint8_t *)timeCapsule[i], strlen(timeCapsule[i]));
		  memory += 4096;
	}
}

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
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

   	  uint8_t rcvBuf;
  	  HAL_StatusTypeDef result;
  	  result = HAL_UART_Receive(&huart3, &rcvBuf, 1, 1000);

  	  if(counter_for_menu){
  	  HAL_UART_Transmit(&huart3, (uint8_t *)"\033[2J", strlen("\033[2J"),10);      //Putty screen clear
  	  HAL_UART_Transmit(&huart3, (uint8_t *)"\r\nMenu:\r\npress 1 to read flash\r\npress 2 to write in flash\r\npress 3 to erase flash\r\n",strlen("\r\nMenu:\r\npress 1 to read flash\r\npress 2 to write in flash\r\npress 3 to erase flash\r\n"),1000);
  	  counter_for_menu--;
  	  }

	  	  if (result == HAL_OK){

  		  switch(rcvBuf){

  			  case '1':
  				  flash_read();
  				  HAL_UART_Transmit(&huart3, (uint8_t *)"Flash is read\r\n", strlen("Flash is read\r\n"),10);
  			  break;

  			  case '2':
  				 flash_write_text();
				 HAL_UART_Transmit(&huart3, (uint8_t *)"Write info in Flash compeleted\r\n", strlen("Write info in Flash compeleted\r\n"),10);
			  break;

  			  case '3':
  				 flash_clear();
				 HAL_UART_Transmit(&huart3, (uint8_t *)"Flash erased\r\n", strlen("Flash erased\r\n"),10);
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
