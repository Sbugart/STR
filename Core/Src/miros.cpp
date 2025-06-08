/****************************************************************************
* MInimal Real-time Operating System (MiROS), GNU-ARM port.
*
* This software is a teaching aid to illustrate the concepts underlying
* a Real-Time Operating System (RTOS). The main goal of the software is
* simplicity and clear presentation of the concepts, but without dealing
* with various corner cases, portability, or error handling. For these
* reasons, the software is generally NOT intended or recommended for use
* in commercial applications.
*
* Copyright (C) 2018 Miro Samek. All Rights Reserved.
*
* SPDX-License-Identifier: GPL-3.0-or-later
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <https://www.gnu.org/licenses/>.
*
* Git repo:
* https://github.com/QuantumLeaps/MiROS
****************************************************************************/
#include <cstdint>
#include <random>
#include "miros.h"
#include "qassert.h"
#include "stm32g4xx.h"
#include "atomic_queue.hpp"

Q_DEFINE_THIS_FILE

namespace rtos {

OSThread * volatile OS_curr; /* pointer to the current thread */
OSThread * volatile OS_next; /* pointer to the next thread to run */

OSThread *OS_thread[32 + 1]; /* array of threads started so far */
uint32_t OS_readySet; /* bitmask of threads that are ready to run */
uint8_t OS_threadNum; /* number of threads started */
uint8_t OS_currIdx; /* current thread index for the circular array */

OSThread *OS_aperiodic_thread[32];
uint8_t OS_aperiodic_threadNum;
AtomicQueue128 OS_aperiodic_queue;




static int32_t OS_random_int32(uint32_t max_magnitude) {
    static std::mt19937 gen(0x12345678U);
    static std::uniform_int_distribution<int32_t> dist;
    
    dist = std::uniform_int_distribution<int32_t>(
        -(int32_t)max_magnitude, 
        (int32_t)max_magnitude
    );
    
    return dist(gen);
}


OSThread idleThread;
void main_idleThread() {
    while (1) {
        OS_onIdle();
    }
}

void OS_init(void *stkSto, uint32_t stkSize) {
    /* set the PendSV interrupt priority to the lowest level 0xFF */
    *(uint32_t volatile *)0xE000ED20 |= (0xFFU << 16);

    /* start idleThread thread */
    OSThread_start(&idleThread,
                   &main_idleThread,
                   stkSto, 400, 0, stkSize);
}

void OS_sched(void) {
    if (OS_readySet == 0U) { /* idle condition? */
    	OS_currIdx = 0U; /* the idle thread */
    } else {
        OS_currIdx = 0U;
    	do{ /* find the next ready thread*/
            OS_currIdx++;
            if(OS_currIdx == OS_threadNum){
            	OS_currIdx = 1;
            }
            OS_next = OS_thread[OS_currIdx];
    	}while((OS_readySet & (1U <<(OS_currIdx - 1U))) == 0 );
    }
    OS_next = OS_thread[OS_currIdx];

    /* trigger PendSV, if needed */
    if(OS_next != OS_curr){
    	*(uint32_t volatile *)0xE000ED04 = (1U << 28);
    }
}

void OS_run(void) {
    /* callback to configure and start interrupts */
    OS_onStartup();

    __disable_irq();
    OS_sched();
    __enable_irq();

    /* the following code should never execute */
    Q_ERROR();
}

void OS_tick(void) {
    static uint32_t ticks = 0U;
    ticks++;
    OSThread *curr_thread;

	uint8_t n = 0;
	for(n=1U;n<OS_threadNum; n++){ 				/* cycle through every thread but the idle */
        curr_thread = OS_thread[n];
        if(curr_thread->timeout != 0U){
			curr_thread->timeout--;			/* decrease the timeout */
            if(curr_thread->timeout == 0U){
                OS_readySet |= (1U << (n-1U));
            }
		}
		if(ticks % curr_thread->period_ticks == 0U){     /*se começou um novo periodo*/
			if(curr_thread->finished_execution){
				curr_thread->finished_execution = false;
				OS_readySet |= (1U << (n-1U));
			} else {
				Q_ERROR();                      /* Deadline acabou antes da thread terminar seu job, causa um erro */
			}
        }
    }
    for(n=0U;n<OS_aperiodic_threadNum;n++){     /* Passa por todas as tarefas aperiódicas */
        curr_thread = OS_aperiodic_thread[n];
        if (ticks % curr_thread->period_ticks + curr_thread->current_period_deviation == 0U) {
            curr_thread->current_period_deviation = OS_random_int32(curr_thread->max_random_period_deviation);  /* Escolhe um novo período aleatório */
            OS_aperiodic_queue.write(n);        /* Coloca a tarefa na fila */
        }
    }
}

void OS_delay(uint32_t ticks) {
    __asm volatile ("cpsid i");

    /* never call OS_delay from the idleThread */
    Q_REQUIRE(OS_curr != OS_thread[0]);     /* Não permite chamadas de delay em tarefas aperiódicas */

    OS_curr->timeout = ticks;
    OS_readySet &= ~(1U << (OS_currIdx - 1U));
    OS_sched();
    __asm volatile ("cpsie i");
}

static void OS_insert_thread(OSThread *me) {
    uint8_t n = 0;
    OSThread *temp = nullptr;
    for(n=1U;n<OS_threadNum; n++){ 				/* cycle through every thread but the idle */
        if(me->period_ticks < OS_thread[n]->period_ticks){ /* check if the new thread has a smaller period */
            temp = OS_thread[n];
            OS_thread[n] = me;
            me = temp;
        }
    }
    OS_thread[OS_threadNum] = me;
    OS_threadNum ++;
}

static void thread_wrapper() {
    // Obter o ponteiro para o thread atual
    OSThread* current = OS_curr;
    
    // Obter o ponteiro para a função original
    OSThreadHandler originalHandler = (OSThreadHandler)current->start_pc;
    
    while (1) {
        // Marcar que o thread começou a executar
        current->finished_execution = false;
        
        // Chamar a função original do thread
        originalHandler();
        
        // Marcar que o thread terminou a execução
        current->finished_execution = true;

        // Remover esta thread da lista de prontas
        __asm volatile ("cpsid i");
        OS_readySet &= ~(1U << (OS_currIdx - 1U));
        OS_sched();
        __asm volatile ("cpsie i");
    }
}

void OSThread_start(
    OSThread *me,
    OSThreadHandler threadHandler,
    void *stkSto, uint32_t period_ticks, uint32_t max_random_period_deviation, uint32_t stkSize)
{
    /* round down the stack top to the 8-byte boundary
    * NOTE: ARM Cortex-M stack grows down from hi -> low memory
    */
    uint32_t *sp = (uint32_t *)((((uint32_t)stkSto + stkSize) / 8) * 8);
    uint32_t *stk_limit;

    /* thread number must be in ragne
    * and must be unused
    */
    Q_REQUIRE((OS_threadNum < Q_DIM(OS_thread)) && (OS_thread[OS_threadNum] == (OSThread *)0));

    *(--sp) = (1U << 24);  /* xPSR */
    if(max_random_period_deviation) *(--sp) = (uint32_t)threadHandler;
    else *(--sp) = (uint32_t)thread_wrapper; /* PC */
    *(--sp) = 0x0000000EU; /* LR  */
    *(--sp) = 0x0000000CU; /* R12 */
    *(--sp) = 0x00000003U; /* R3  */
    *(--sp) = 0x00000002U; /* R2  */
    *(--sp) = 0x00000001U; /* R1  */
    *(--sp) = 0x00000000U; /* R0  */
    /* additionally, fake registers R4-R11 */
    *(--sp) = 0x0000000BU; /* R11 */
    *(--sp) = 0x0000000AU; /* R10 */
    *(--sp) = 0x00000009U; /* R9 */
    *(--sp) = 0x00000008U; /* R8 */
    *(--sp) = 0x00000007U; /* R7 */
    *(--sp) = 0x00000006U; /* R6 */
    *(--sp) = 0x00000005U; /* R5 */
    *(--sp) = 0x00000004U; /* R4 */

    /* save the top of the stack in the thread's attibute */
    me->sp = sp;

    /* save the period in the thread's attribute */
    me->period_ticks = period_ticks;

    me->finished_execution = false; /* mark the thread as not finished */

    me->start_pc = (uint32_t)threadHandler;

    me->max_random_period_deviation = max_random_period_deviation; /* Simulation of aperiodic tasks */

    me->current_period_deviation = 0;

    /* round up the bottom of the stack to the 8-byte boundary */
    stk_limit = (uint32_t *)(((((uint32_t)stkSto - 1U) / 8) + 1U) * 8);

    /* pre-fill the unused part of the stack with 0xDEADBEEF */
    for (sp = sp - 1U; sp >= stk_limit; --sp) {
        *sp = 0xDEADBEEFU;
    }

    if (max_random_period_deviation) {
        OS_aperiodic_thread[OS_aperiodic_threadNum] = me;
        OS_aperiodic_threadNum ++;
        return;
    }
    /* register the thread with the OS */
    OS_insert_thread(me);

    /* make the thread ready to run */
    /* só funciona porque todas as threads são criadas antes de rodar o sistema */
    if (OS_threadNum > 1U) {
        OS_readySet |= (1U << (OS_threadNum - 2U));
    }
}
/***********************************************/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    //Default_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    //Default_Handler();
  }
}

void OS_onStartup(void) {
	SystemClock_Config();
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / TICKS_PER_SEC);

    /* set the SysTick interrupt priority (highest) */
    NVIC_SetPriority(SysTick_IRQn, 0U);
}

void OS_onIdle(void) {
#ifdef NDBEBUG
    __WFI(); /* stop the CPU and Wait for Interrupt */
#endif
    int n;

    __asm volatile ("cpsid i");             /* impede que um OS_Tick aconteça enquanto fila está sendo lida */
    int result = OS_aperiodic_queue.read(n);        /* Se tiver thread aperiodica na fila, execute*/
    __asm volatile ("cpsie i");

    if(result == 0) {
        ((OSThreadHandler)OS_aperiodic_thread[n]->start_pc)();
    }
}

}//fim namespace

void Q_onAssert(char const *module, int loc) {
    /* TBD: damage control */
    (void)module; /* avoid the "unused parameter" compiler warning */
    (void)loc;    /* avoid the "unused parameter" compiler warning */
    NVIC_SystemReset();
}

/***********************************************/
__attribute__ ((naked, optimize("-fno-stack-protector")))
void PendSV_Handler(void) {
__asm volatile (

    /* __disable_irq(); */
    "  CPSID         I                 \n"

    /* if (OS_curr != (OSThread *)0) { */
    "  LDR           r1,=_ZN4rtos7OS_currE       \n"
    "  LDR           r1,[r1,#0x00]     \n"
    "  CBZ           r1,PendSV_restore \n"

    /*     push registers r4-r11 on the stack */
    "  PUSH          {r4-r11}          \n"

    /*     OS_curr->sp = sp; */
    "  LDR           r1,=_ZN4rtos7OS_currE       \n"
    "  LDR           r1,[r1,#0x00]     \n"
    "  STR           sp,[r1,#0x00]     \n"
    /* } */

    "PendSV_restore:                   \n"
    /* sp = OS_next->sp; */
    "  LDR           r1,=_ZN4rtos7OS_nextE       \n"
    "  LDR           r1,[r1,#0x00]     \n"
    "  LDR           sp,[r1,#0x00]     \n"

    /* OS_curr = OS_next; */
    "  LDR           r1,=_ZN4rtos7OS_nextE       \n"
    "  LDR           r1,[r1,#0x00]     \n"
    "  LDR           r2,=_ZN4rtos7OS_currE       \n"
    "  STR           r1,[r2,#0x00]     \n"

    /* pop registers r4-r11 */
    "  POP           {r4-r11}          \n"

    /* __enable_irq(); */
    "  CPSIE         I                 \n"

    /* return to the next thread */
    "  BX            lr                \n"
    );
}
