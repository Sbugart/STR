/*
 * prodcons.h
 *
 *  Created on: May 14, 2025
 *      Author: artur
 */

#ifndef INC_PRODCONS_H_
#define INC_PRODCONS_H_
#include "semaforo.h"
#include "main.h"
#include "miros.h"

struct ProdCons{
	static constexpr int BUFFER_SIZE = 5;

	Semaforo emptySlots;
	Semaforo fullSlots;
	Semaforo bufferMutex;

	int32_t buffer[BUFFER_SIZE];
	int32_t head;
	int32_t tail;

	GPIO_TypeDef* LED_PORT;
    uint16_t PROD_LED_PIN;
    uint16_t CONS_LED_PIN;

    uint32_t stack_produtor[100];
    uint32_t stack_consumidor[100];
    uint32_t stack_mutex[100];

    rtos::OSThread thread_produtor;
    rtos::OSThread thread_consumidor;
    rtos::OSThread thread_mutex;

    void init(GPIO_TypeDef* port, uint16_t prod_pin, uint16_t cons_pin);
    void produtor();
    void thread_produtor_wrapper(void* arg);
    void consumidor();
    void thread_consumidor_wrapper(void* arg);
};


#endif /* INC_PRODCONS_H_ */
