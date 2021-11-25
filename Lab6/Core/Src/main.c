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
#define SLAVE_ADDRESS 0x80  //PCA9685 I2C bus address   0b10000000
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

/* USER CODE BEGIN PV */

uint8_t TxBuffer[2];

//uint8_t arr_led_on[4]={0x00,0x10,0x00,0x00};//all led on
//4096 to bin 0b00010000(ALL_LED_ON_H) 00000000(ALL_LED_ON_L)-{0x00,0x10} first two elements





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void all_led_off(I2C_HandleTypeDef *hi2c,uint8_t TxBuffer[]){

	  uint8_t arr_led_off[4]={0x00,0x00,0x00,0x10}; //all led off
	  TxBuffer[0]=0xFA;
	  for(uint8_t i=0;i<4;i++){
		  TxBuffer[1]=arr_led_off[i];
		  HAL_I2C_Master_Transmit(hi2c, SLAVE_ADDRESS, (uint8_t *)TxBuffer, 2, WAITTIME);
		  TxBuffer[0]++;
	  }
}


void all_led_on(I2C_HandleTypeDef *hi2c,uint8_t TxBuffer[]){

	  uint8_t arr_led_on[4]={0x00,0x10,0x00,0x00};//all led on
	  TxBuffer[0]=0xFA;
	  for(uint8_t i=0;i<4;i++){
		  TxBuffer[1]=arr_led_on[i];
		  HAL_I2C_Master_Transmit(hi2c, SLAVE_ADDRESS, (uint8_t *)TxBuffer, 2, WAITTIME);
		  TxBuffer[0]++;
	  }
}





//bool pcaEnableSleepMode(I2C_HandleTypeDef hi2c)
//{
//	uint8_t register0;
//	if (!readRegister(hi2c, 0x00, &register0))
//	{
//		return 0;
//	}
//
//	register0 |= 0b00010000; // sleep bit
//	return setRegister(hi2c, 0x00, register0);
//}
//
//static bool readRegister(I2C_HandleTypeDef hi2c, uint8_t adress, uint8_t *save)
//{
//	HAL_I2C_Master_Transmit(&hi2c, ADRESS, (uint8_t *)&adress, 1, WAITTIME);
//	uint8_t result = HAL_I2C_Master_Receive(&hi2c, ADRESS, (uint8_t *)save, 1, WAITTIME) == HAL_OK;
//	return result;
//}


//bool pcaSetDutyCycle(I2C_HandleTypeDef hi2c, uint8_t newDutyCycle)
//{
//
//		if (newDutyCycle <= 100 && newDutyCycle >= 0)
//		{
//			uint16_t setOn = newDutyCycle == 100 ? 4096 : 0;
//			uint16_t setOff = newDutyCycle == 100 ? 0 : round(((float)newDutyCycle / 100) * 4096);
//
////			uint8_t finalChannel = 0xFA; //all leds
//			uint8_t data4Elements[4] = {setOn, setOn >> 8u, setOff, setOff >> 8u};
//			return setPin(data4Elements);
//		}
//
//	return 0;
//}
//
//
//void setPin(uint8_t *data4Elements){
//	  int8_t TxBuffer[2];
//	  TxBuffer[0]=0xFA;
//	  for(uint8_t i=0;i<4;i++){
//		  TxBuffer[1]=data4Elements[i];
//		  HAL_I2C_Master_Transmit(&hi2c1, ADRESS, (uint8_t *)&TxBuffer, 2, 1000);
//		  TxBuffer[0]++;
//	  } //PWM start
//}


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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);   //turn on of the PB7 (output enable signal)


//  all_led_off(&hi2c1,arr_led_off,TxBuffer);

//  uint8_t arr_led_off[4]={0x00,0x00,0x00,0x10};

//  TxBuffer[0]=0xFA;
//  for(uint8_t i=0;i<4;i++){
//	  TxBuffer[1]=arr_led_off[i];
//	  HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS, (uint8_t *)&TxBuffer, 2, WAITTIME);
//	  TxBuffer[0]++;
//  }



//    uint8_t arr_led_oN[4]={0x00,0x10,0x00,0x00};
//
//    TxBuffer[0]=0xFA;
//    for(uint8_t i=0;i<4;i++){
//  	  TxBuffer[1]=arr_led_oN[i];
//  	  HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS, (uint8_t *)&TxBuffer, 2, WAITTIME);
//  	  TxBuffer[0]++;
//    }

    all_led_on(&hi2c1,TxBuffer);
//    all_led_off(&hi2c1,TxBuffer);





//  uint8_t data4Elements[4]={0x00,0x00,0x00,0x10}; //all led off
//
//  uint8_t data4Elements[4]={0xE8,0x3,0xD0,0x7}; //PWM signal

//  1000 to bin 0b 00000011 11101000  for PWM
//  2000 to bin 0b 00000111 11010000
//
//  uint8_t prescale_value=0x1E;  //200Hz
//
//
//
//
//PWM start
//
//  TxBuffer[0]=0xFE;
//	TxBuffer[1]=prescale_value;
//	HAL_I2C_Master_Transmit(&hi2c1, devId, (uint8_t *)&TxBuffer, 2, 1000);
//frequency change (to change frequency we need some PWM)


//	pcaEnableSleepMode(hi2c1);
//	pcaSetDutyCycle(hi2c1,50);


  while (1)
  {
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
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
