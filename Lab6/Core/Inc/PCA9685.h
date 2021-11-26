/*
 * PCA9685.h
 *
 *  Created on: Nov 26, 2021
 *      Author: Hrona Yurii
 */

#ifndef INC_PCA9685_H_
#define INC_PCA9685_H_

/* ------------------------ Includes------------------------------------------*/
#include <stdio.h>
#include "stm32f4xx_hal.h"

void arr_filler_and_transmit(I2C_HandleTypeDef *hi2c,uint8_t TxBuffer[],uint16_t arr[]);
void all_led_off(I2C_HandleTypeDef *hi2c,uint8_t TxBuffer[]);
void all_led_on(I2C_HandleTypeDef *hi2c,uint8_t TxBuffer[]);
void pwm_on(I2C_HandleTypeDef *hi2c,uint8_t TxBuffer[]);
void change_duty_cycle(uint8_t newDutyCycle,I2C_HandleTypeDef *hi2c,uint8_t TxBuffer[]);
void bit_filler_and_transmit(I2C_HandleTypeDef *hi2c, uint8_t Reg, uint32_t Data);
void sleep_mode(I2C_HandleTypeDef *hi2c, uint8_t flag);
uint32_t Round_Int_To(uint32_t arg, uint32_t divider);
void PCA9685_Frequency_Set(I2C_HandleTypeDef *hi2c, uint16_t frequency);


#endif /* INC_PCA9685_H_ */
