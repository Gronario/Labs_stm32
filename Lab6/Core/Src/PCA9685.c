/*
 * PCA9685.c
 *
 *  Created on: Nov 26, 2021
 *      Author: Hrona Yurii
 */

/*  ------------------------Private includes-----------------------------------------*/

#include "PCA9685.h"
#include "math.h"

/*  ------------------------Private variables----------------------------------------*/

extern uint8_t MODE0_reg;
extern uint8_t PRE_SCALE_reg;
#define WAITTIME 1000
#define SLAVE_ADDRESS 0x80
extern I2C_HandleTypeDef hi2c1;

/*  -------------------------Library code begin--------------------------------------*/

void arr_filler_and_transmit(I2C_HandleTypeDef *hi2c,uint8_t TxBuffer[],uint16_t arr[]){

	  TxBuffer[0]=0xFA;
	  for(uint8_t i=0;i<4;i++){
		  TxBuffer[1]=arr[i];
		  HAL_I2C_Master_Transmit(hi2c, SLAVE_ADDRESS, (uint8_t *)TxBuffer, 2, WAITTIME);
		  TxBuffer[0]++;
       }
}

void all_led_off(I2C_HandleTypeDef *hi2c,uint8_t TxBuffer[]){

	  uint16_t arr_led_off[4]={0x00,0x00,0x00,0x10}; //all led off
	  arr_filler_and_transmit(&hi2c1,TxBuffer, arr_led_off);
}


void all_led_on(I2C_HandleTypeDef *hi2c,uint8_t TxBuffer[]){

	  //uint8_t arr_led_on[4]={0x00,0x10,0x00,0x00};//all led on
	  //4096 to bin 0b00010000(ALL_LED_ON_H) 00000000(ALL_LED_ON_L) => {0x00(ALL_LED_ON_L),0x10(ALL_LED_ON_H)} first  two elements
	  //   0 to bin 0b00000000(ALL_LED_ON_H) 00000000(ALL_LED_ON_L) => {0x00(ALL_LED_ON_L),0x00(ALL_LED_ON_H)} second two elements

	  uint16_t arr_led_on[4]={0x00,0x10,0x00,0x00};//all led on
	  arr_filler_and_transmit(&hi2c1,TxBuffer, arr_led_on);
}

void pwm_on(I2C_HandleTypeDef *hi2c,uint8_t TxBuffer[]){

	  //  1000 to bin 0b 00000011 11101000 => 0xE8 0x3
	  //  2000 to bin 0b 00000111 11010000 => 0xD0,0x7

	  uint16_t arr_pwm_on[4]={0xE8,0x3,0xD0,0x7};   //some kind of PWM signal
	  arr_filler_and_transmit(&hi2c1,TxBuffer, arr_pwm_on);
}


void change_duty_cycle(uint8_t newDutyCycle,I2C_HandleTypeDef *hi2c,uint8_t TxBuffer[]){

	 uint16_t setOn = newDutyCycle == 100 ? 4096 : 0;
	 uint16_t setOff = newDutyCycle == 100 ? 0 : round(((float)newDutyCycle / 100) * 4096);
	 uint16_t arr_duty_cycle[4] = {setOn, setOn >> 8u, setOff, setOff >> 8u};
	 arr_filler_and_transmit(&hi2c1,TxBuffer, arr_duty_cycle);
}

void bit_filler_and_transmit(I2C_HandleTypeDef *hi2c, uint8_t Reg, uint32_t Data)
{
	uint8_t TxBuffer[2];
	TxBuffer[0] = Reg;
	TxBuffer[1] = Data;
	HAL_I2C_Master_Transmit(hi2c, SLAVE_ADDRESS, (uint8_t*) &TxBuffer, 2, 1000);
}

void sleep_mode(I2C_HandleTypeDef *hi2c, uint8_t flag)
{
	uint8_t MODE0_reg = 0x00;
	if (flag == 1)
	{
		bit_filler_and_transmit(hi2c, MODE0_reg, 0x10);  // enable Sleep mode: set Sleep bit D4 in 1
	}
	else
	{
		HAL_Delay(10);
		bit_filler_and_transmit(hi2c, MODE0_reg, 0x01); //disable Sleep mode: set Sleep bit D4 in 0
	}
}

uint32_t Round_Int_To(uint32_t arg, uint32_t divider)
{
	if (arg % divider < (divider / 2))
		return arg - arg % divider;
	else if (arg % divider >= (divider / 2))
		return arg - arg % divider + divider;
	else
		return arg;
}

void PCA9685_Frequency_Set(I2C_HandleTypeDef *hi2c, uint16_t frequency)
{

	uint32_t prescale = Round_Int_To((25000000 / (frequency << 12) * 10), 10) / 10 - 1; // = 25MHz /(4096 * frequency) -1, where 2500000 - 25 MHz internal oscillator clock frequency.
	prescale = (frequency == 0) ? 0x1E : prescale; //set default 200Hz if frequency argument is 0
	prescale = (prescale > 0xFF) ? 0xFF : prescale;
	prescale = (prescale < 0x03) ? 0x03 : prescale;

	uint8_t TxBuffer[2];
	TxBuffer[0] = MODE0_reg;
	TxBuffer[1] = 0x21; // enable AI
	HAL_I2C_Master_Transmit(hi2c, SLAVE_ADDRESS , (uint8_t*) &TxBuffer, 2, 10);

	uint8_t Buffer[70] ={ 0 };

	HAL_I2C_Master_Receive(hi2c, SLAVE_ADDRESS , (uint8_t*) &Buffer, 70, 100); //read and store all registers from the device into an array
	for (int i = sizeof(Buffer) - 1; i > 0; i--)
		Buffer[i] = Buffer[i - 2];
	Buffer[0] = MODE0_reg;
	Buffer[1] = 0x21;

	TxBuffer[0] = MODE0_reg;
	TxBuffer[1] = 0x10; // enable Sleep mode to change PRE_SCAL: set Sleep bit D4 in 1.
	HAL_I2C_Master_Transmit(hi2c, SLAVE_ADDRESS , (uint8_t*) &TxBuffer, 2, 10);

	TxBuffer[0] = PRE_SCALE_reg;
	TxBuffer[1] = prescale; //set frequency
	HAL_I2C_Master_Transmit(hi2c, SLAVE_ADDRESS , (uint8_t*) &TxBuffer, 2, 10); //   load all settings from the array..
	HAL_I2C_Master_Transmit(hi2c, SLAVE_ADDRESS , (uint8_t*) &Buffer, 70, 100); // ..into the device registers. Disable sleep mode, start with normal mode.
}
