/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

  #include "main.h"
  #include <cstdint>
  #include "miros.h"

  #include "mutex.h"
  #include "semaforo.h"
  #include "prodcons.h"

  extern "C"{
  	  #include "config_micro.h"
  }
  #include "motor.h"

  ProdCons pc;

  uint32_t conta0=0, conta1=0, conta2=0,conta3 = 0,
		  conta4 = 0;

  int32_t critical_variable = 0, global_use = 0;

  uint32_t stack_blinky1[40];
  rtos::OSThread blinky1;
  void main_blinky1() {
	  conta0++;

	  rtos::OS_enterCriticalSection();
	  critical_variable = 2;
	  global_use = 0;
	  for(int32_t i = 0; i < 10; ++i){
		  global_use += critical_variable;
	  }

	  rtos::OS_exitCriticalSection();
  }

  uint32_t stack_blinky2[40];
  rtos::OSThread blinky2;
  void main_blinky2() {
	  conta1++;

	  rtos::OS_enterCriticalSection();
	  critical_variable = 5;
	  global_use = 0;
	  for(int32_t i = 0; i < 10; ++i){
		  global_use -= critical_variable;
	  }

	  rtos::OS_exitCriticalSection();
  }

  uint32_t stack_blinky3[40];
  rtos::OSThread blinky3;
  void main_blinky3() {
	  conta2++;
  }


  uint32_t stack_blinky4[40];
  rtos::OSThread blinky4;
  void main_blinky4() {

		 if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET)
		 {
		   /* botão pressionado → acende LED */
		   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		   Motor_SetPower(50);
		 }
		 else
		 {
		   /* botão solto → apaga LED */
			 Motor_SetPower(60);
		   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		 }

  }



  uint32_t stack_blinky5[40];
  rtos::OSThread blinky5;
  void main_blinky5() {
	  conta4++;
  }

  uint32_t stack_aperiodicas[40];
  rtos::OSThread aperiodicas;
  void main_aperiodicas(){
	  rtos::OSThread_start(&blinky5,
			   &main_blinky5,
			   stack_blinky5, sizeof(stack_blinky5),0U, 0U, rtos::TICKS_PER_SEC, false);
  }



  uint32_t stack_idleThread[40];

  void consumidor_wrapper() {
  	pc.consumidor();
  }

  void produtor_wrapper(){
	  pc.produtor();
  }

  uint32_t stack_aperiodicThread[40];

int main(void)
  {
	uint32_t aperiodic_period = rtos::TICKS_PER_SEC / 4U;
	uint32_t aperiodic_capacity = 4;

	rtos::OS_init(stack_idleThread, sizeof(stack_idleThread), aperiodic_period, aperiodic_capacity,
			stack_aperiodicThread, sizeof(stack_aperiodicThread));


	HAL_Init();
	SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM20_Init();

	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


	HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_2);
	Motor_SetPower(60);

	/*while(1){
		 if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET)
		 {

		   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		   Motor_SetPower(50);
		 }
		 else
		 {
			 Motor_SetPower(60);
		   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		 }
	}*/
	//pc.init(GPIOA, GPIO_PIN_5, GPIO_PIN_6);

	/*
 	----       Pino
	PWM        PC2
	SDA        PB7
	SLC        PA15
	 */


	/*rtos::OSThread_start(&blinky1,
			   &main_blinky1,
			   stack_blinky1, sizeof(stack_blinky1), rtos::TICKS_PER_SEC / 10U, 0U, rtos::TICKS_PER_SEC / 10U, false);



	rtos::OSThread_start(&blinky2,
			   &main_blinky2,
			   stack_blinky2, sizeof(stack_blinky2), rtos::TICKS_PER_SEC / 4U, 0U, rtos::TICKS_PER_SEC / 10U, false);


	rtos::OSThread_start(&aperiodicas,
			   &main_aperiodicas,
			   stack_aperiodicas, sizeof(stack_aperiodicas),rtos::TICKS_PER_SEC / 4U, 0U, rtos::TICKS_PER_SEC, false);


    rtos::OSThread_start(&blinky5,
		   &main_blinky5,
		   stack_blinky5, sizeof(stack_blinky5),0U, 0U, rtos::TICKS_PER_SEC, false);

	rtos::OSThread_start(&blinky3,
			   &main_blinky3,
			   stack_blinky3, sizeof(stack_blinky3), rtos::TICKS_PER_SEC / 2U, 0U, rtos::TICKS_PER_SEC, false);

	 */

	rtos::OSThread_start(&blinky4,
			   &main_blinky4,
			   stack_blinky4, sizeof(stack_blinky4), rtos::TICKS_PER_SEC, 0U, rtos::TICKS_PER_SEC, false);

	rtos::OS_run();

}
