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
#include <stdio.h>
#include "atomic_queue.hpp"

AtomicQueue128 queue;

uint32_t stack_prod1[40];
rtos::OSThread prod1;
void main_prod1() {
    queue.write(1);
}

uint32_t stack_prod2[40];
rtos::OSThread prod2;
void main_prod2() {
    queue.write(2);
}

uint32_t stack_prod3[40];
rtos::OSThread prod3;
void main_prod3() {
    queue.write(3);
}

uint32_t stack_aperiodic1[40];
rtos::OSThread aperiodic1;
void main_aperiodic1() {
    queue.write(101);
}

uint32_t stack_aperiodic2[40];
rtos::OSThread aperiodic2;
void main_aperiodic2() {
    queue.write(102);
}

uint32_t stack_idleThread[1024];

int main(void)
{
    rtos::OS_init(stack_idleThread, sizeof(stack_idleThread));

    /* Tarefas Periódicas*/
    rtos::OSThread_start(&prod1,
                     &main_prod1,
                     stack_prod1, 10, 0, sizeof(stack_prod1));
    
    rtos::OSThread_start(&prod2,
                     &main_prod2,
                     stack_prod2, 5, 0, sizeof(stack_prod2));
                     
    rtos::OSThread_start(&prod3,
                     &main_prod3,
                     stack_prod3, 3, 0, sizeof(stack_prod3));

    /* Tarefas Aperiódicas */
    rtos::OSThread_start(&aperiodic1,
                     &main_aperiodic1,
                     stack_aperiodic1, 10, 3, sizeof(stack_aperiodic1));
    
    rtos::OSThread_start(&aperiodic2,
                     &main_aperiodic2,
                     stack_aperiodic2, 20, 10, sizeof(stack_aperiodic2));
                    
    rtos::OS_run();
}
