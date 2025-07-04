/*
 * prodcons.cpp
 *
 *  Created on: May 14, 2025
 *      Author: artur
 */

#include "prodcons.h"

void ProdCons::init(GPIO_TypeDef* port, uint16_t prod_pin, uint16_t cons_pin){
	LED_PORT = port;
	PROD_LED_PIN = prod_pin;
	CONS_LED_PIN = cons_pin;

	head = 0; tail = 0;

	emptySlots.init(BUFFER_SIZE, BUFFER_SIZE);
	fullSlots.init(0, BUFFER_SIZE);
	bufferMutex.init(1, 1);
}


void ProdCons::produtor(){
    uint32_t valor = 0;
    while (1) {
    	valor = (valor + 1) % 5;

        emptySlots.wait();
        bufferMutex.wait();

        buffer[head] = valor;
        head = (head + 1) % BUFFER_SIZE;

        HAL_GPIO_WritePin(LED_PORT, PROD_LED_PIN, GPIO_PIN_SET);
        rtos::OS_delay(rtos::TICKS_PER_SEC / 2);
        HAL_GPIO_WritePin(LED_PORT, PROD_LED_PIN, GPIO_PIN_RESET);

        fullSlots.signal();
        bufferMutex.signal();
    }
}


void ProdCons::consumidor(){
  while (1) {
	  fullSlots.wait();
	  bufferMutex.wait();

	  int dado = buffer[tail];
	  tail = (tail + 1) % BUFFER_SIZE;

	  for (int i = 0; i < dado + 5; i++) {
		  HAL_GPIO_TogglePin(LED_PORT, CONS_LED_PIN);
		  rtos::OS_delay(rtos::TICKS_PER_SEC / 10);
	  }

	  bufferMutex.signal();
	  emptySlots.signal();
  }
}


