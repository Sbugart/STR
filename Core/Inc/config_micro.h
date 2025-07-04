/*
 * config_micro.h
 *
 *  Created on: Jul 3, 2025
 *      Author: artur
 */

#ifndef INC_CONFIG_MICRO_H_
#define INC_CONFIG_MICRO_H_

#include "stm32g4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim20;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM20_Init(void);
void MX_I2C1_Init(void);
void Motor_SetPower(uint8_t power_percent);


#endif /* INC_CONFIG_MICRO_H_ */
