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
#include "miros.h"
#include "qassert.h"
#include "stm32g4xx.h"
//#include "tim.h"
#include "stm32g4xx_hal_tim.h"
#include <vector>

Q_DEFINE_THIS_FILE

namespace rtos {

OSThread * volatile OS_curr; /* pointer to the current thread */
OSThread * volatile OS_next; /* pointer to the next thread to run */

OSThread *OS_thread[32 + 1];  // Vetor de ponteiros para threads (posição 0 reservada para idle)
int32_t OS_readySet;          // Bitmask das threads prontas (threads do índice 1 em diante)
uint8_t OS_threadNum = 0;     // Número de threads criadas
uint8_t OS_currIdx;           // Índice da thread atual no vetor OS_thread

uint32_t mod = 1e9;           // Valor para fazer wrap-around dos temporizadores (overflow controlado)
uint32_t timer = 0U;          // Temporizador global baseado no HAL_GetTick()

OSAperiodicThread OS_aperiodic; // Estrutura que gerencia execução de threads aperiódicas
OSThread aperiodic;             // Thread de servidor aperiódico

volatile int32_t OS_criticalNesting = 0; // Contador de nesting de seções críticas

OSThread idleThread;
void main_idleThread() {
    while (true) {
        OS_onIdle();
    }
}

// Thread que executa threads aperiódicas com base em capacidade disponível
void main_aperiodicThread(void){
	while(true){
		if(OS_aperiodic.tam == 0U || OS_aperiodic.capacity == 0U){
			// Nenhuma thread a ser executada OU sem capacidade → reagenda para próximo tick
			OS_aperiodic.timers[OS_aperiodic.timer_end] = (timer + 2U) % mod;
			OS_aperiodic.capacitys[OS_aperiodic.timer_end] = 0U;
			OS_aperiodic.timer_end = (OS_aperiodic.timer_end + 1U) % OS_aperiodic.max_tam;
			break;
		}
		else{
			// Executa uma thread aperiódica pendente
			OS_aperiodic.time_init = HAL_GetTick();
			void (*handler)(void) = (void (*)(void))(OS_aperiodic.OS_thread[OS_aperiodic.init]->handler);
			handler(); // Executa o handler

			__disable_irq();
			OS_aperiodic.time_end = HAL_GetTick();

			// Calcula o tempo de execução consumido
			uint64_t delta = OS_aperiodic.time_end - OS_aperiodic.time_init;
			if(delta == 0U) delta++; // Garantir consumo mínimo

			// Agenda reabastecimento da capacidade no futuro
			OS_aperiodic.capacitys[OS_aperiodic.timer_end] = delta;
			OS_aperiodic.timers[OS_aperiodic.timer_end] = (timer + OS_thread[OS_currIdx]->period) % mod;
			OS_aperiodic.timer_end = (OS_aperiodic.timer_end + 1U) % OS_aperiodic.max_tam;
			OS_aperiodic.capacity -= delta;

			// Atualiza fila circular de execução
			OS_aperiodic.init = (OS_aperiodic.init + 1U) % OS_aperiodic.max_tam;
			OS_aperiodic.tam--;

			__enable_irq();

		}
	}
}

void OS_enterCriticalSection(void) {
	__disable_irq(); // Desativa interrupções
	OS_criticalNesting = OS_criticalNesting + 1;
}

void OS_exitCriticalSection(void) {
	Q_REQUIRE(OS_criticalNesting > 0); // Protege contra chamadas inválidas
	OS_criticalNesting = OS_criticalNesting - 1;
	if(OS_criticalNesting == 0){
		__enable_irq(); // Restaura interrupções se não houver mais nesting
	}
}


void OS_init(void *stkSto, uint32_t stkSize, uint32_t period, uint32_t capacity, void *stkApe, uint32_t stkApeSize) {
    /* set the PendSV interrupt priority to the lowest level 0xFF */
    *(uint32_t volatile *)0xE000ED20 |= (0xFFU << 16);

    /* start idleThread thread */
    OSThread_start(&idleThread,
                   &main_idleThread,
                   stkSto, stkSize, 0, 0, 0, false);

    OS_aperiodic.capacity = capacity;
    OS_aperiodic.period = period;
    OSThread_start(&aperiodic,
                       &main_aperiodicThread,
					   stkApe, stkApeSize,
					   period, 0, rtos::TICKS_PER_SEC, true);
}


void OS_sched(void) {
    if (OS_readySet == 0U) {
        // Nenhuma thread pronta → executa idle (índice 0)
        OS_currIdx = 0U;
    }
    else if(OS_criticalNesting == 0){
    	// Escolhe próxima thread com menor índice (mais prioritária)
    	uint8_t novo = __builtin_ctz(OS_readySet) + 1;

    	// Se thread atual é servidor e vai ser preemptada com aperiodic pendente → salva tempo
    	if(OS_thread[OS_currIdx]->is_server && novo != OS_currIdx && OS_aperiodic.tam != 0){
    		OS_aperiodic.time_end = timer;
			uint32_t delta = OS_aperiodic.time_end - OS_aperiodic.time_init;
			if(delta == 0U) delta++;
			OS_aperiodic.capacitys[OS_aperiodic.timer_end] = delta;
			OS_aperiodic.timers[OS_aperiodic.timer_end] = (timer + OS_thread[OS_currIdx]->period) % mod;
			OS_aperiodic.timer_end = (OS_aperiodic.timer_end + 1U) % OS_aperiodic.max_tam;
			OS_aperiodic.capacity -= delta;
    	}
    	else if(OS_thread[novo]->is_server && novo != OS_currIdx){
    		// Nova thread é servidor → salva tempo de início
    		OS_aperiodic.time_init = timer;
    	}

    	OS_currIdx = novo;
    }

    OS_next = OS_thread[OS_currIdx];

    // Se troca de contexto for necessária → dispara PendSV
    if(OS_next != OS_curr){
    	*(uint32_t volatile *)0xE000ED04 = (1U << 28); // Set PendSV
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

	uint8_t n;
	for(n = 1U; n < OS_threadNum; ++n){

		if(OS_thread[n]->is_server){
			// Reabastecimento da capacidade (baseado no buffer circular de timers)
			if(OS_aperiodic.timer_init != OS_aperiodic.timer_end && OS_aperiodic.timers[OS_aperiodic.timer_init] == timer){
				OS_aperiodic.capacity += OS_aperiodic.capacitys[OS_aperiodic.timer_init];
				OS_aperiodic.timer_init = (OS_aperiodic.timer_init + 1U) % OS_aperiodic.max_tam;
				OS_readySet |= (1U << (n - 1U));
			}
			else if(timer == 0 && OS_aperiodic.timer_init == 0 && OS_aperiodic.timer_end == 0){
				// Inicialização inicial (caso especial)
				OS_readySet |= (1U << (n - 1U));
			}
			else if(OS_currIdx == n && (OS_aperiodic.capacity + OS_aperiodic.time_init <= timer)){
				// Capacidade esgotada → retira do conjunto de prontas
				OS_readySet &= ~(1U << (n - 1U));
			}
		}
		else{
			// Threads periódicas: ativa se chegou a hora
			if(OS_thread[n]->timer == timer){
				OS_readySet |= (1U << (n - 1U));

				// Atualiza temporizadores
				OS_thread[n]->exec_time = (OS_thread[n]->timer + OS_thread[n]->total_time) % mod;
				OS_thread[n]->timer = (OS_thread[n]->timer + OS_thread[n]->period) % mod;
			}
		}
	}

	timer = HAL_GetTick(); // Atualiza temporizador global
}

void OS_delay(uint32_t ticks) {
    __asm volatile ("cpsid i");

    /* never call OS_delay from the idleThread */
    Q_REQUIRE(OS_curr != OS_thread[0]);

    OS_thread[OS_currIdx]->timer = (timer + ticks) % mod;

    OS_readySet &= ~(1U << (OS_currIdx - 1U));
    OS_sched();
    __asm volatile ("cpsie i");
 }

void thread_wrapper(void) {
    void (*handler)(void) = (void (*)(void))(OS_curr->handler);

    while(true){
        handler();

        OS_readySet &= ~(1U << (OS_currIdx - 1U));

        __disable_irq();
        OS_sched();
        __asm volatile ("svc 0");
        __enable_irq();

    }
}

void aperiodic_thread_wrapper(void){
	while(true){
		void (*handler)(void) = (void (*)(void))(OS_aperiodic.OS_thread[OS_aperiodic.init]->handler);
		handler();
		__asm volatile ("svc 0");
	}
}


void insertion_sort(OSThread *me){
	uint8_t pos = OS_threadNum;
	uint32_t newReadySet = OS_readySet;
	while (pos > 0U && OS_thread[pos - 1]->period > me->period) {
		OS_thread[pos] = OS_thread[pos - 1];

		if(OS_readySet & (1U << (pos - 2))){
			newReadySet |= (1U << (pos - 1));
		}
		else{
			newReadySet &= ~(1U << (pos - 1));
		}
		--pos;
	}
	OS_thread[pos] = me;

    /* make the thread ready to run */
    /*if (pos > 0U) {
    	newReadySet |= (1U << (pos - 1U));
    }*/
    OS_readySet = newReadySet;
}

void OSThread_start(
    OSThread *me,
    OSThreadHandler threadHandler,
    void *stkSto, uint32_t stkSize,
	uint32_t period, uint32_t release,
    uint32_t total_time,
	bool is_server)
{
    /* round down the stack top to the 8-byte boundary
    * NOTE: ARM Cortex-M stack grows down from hi -> low memory
    */
	__disable_irq();

    uint32_t *sp = (uint32_t *)((((uint32_t)stkSto + stkSize) / 8) * 8);
    uint32_t *stk_limit;

    /* thread number must be in ragne
    * and must be unused
    */

    Q_REQUIRE((OS_threadNum < Q_DIM(OS_thread)) && (OS_thread[OS_threadNum] == (OSThread *)0));

    *(--sp) = (1U << 24);  /* xPSR */
    //*(--sp) = (uint32_t)threadHandler; /* PC */
    if(period == 0U && OS_threadNum != 0U){
    	*(--sp) = (uint32_t)&aperiodic_thread_wrapper;
    }
    else{
    	*(--sp) = (uint32_t)&thread_wrapper;
    }
    //*(--sp) = (uint32_t)&thread_wrapper;
    *(--sp) = 0x0000000EU; /* LR  */
    //*(--sp) = (uint32_t)&teste;
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

    /* save the top of the stack in the thread's attribute */
    me->sp = sp;
    me->sp_base = sp;

    /* round up the bottom of the stack to the 8-byte boundary */
    stk_limit = (uint32_t *)(((((uint32_t)stkSto - 1U) / 8) + 1U) * 8);

    /* pre-fill the unused part of the stack with 0xDEADBEEF */
    for (sp = sp - 1U; sp >= stk_limit; --sp) {
        *sp = 0xDEADBEEFU;
    }
    me->period = period;
    me->release = release;
    me->timer = release;
    me->total_time = total_time;
    me->exec_time = 0;
    me->handler = (uint32_t)threadHandler;
    me->is_server = is_server;


    /* Para colocar na fila das Thread aperiodicas*/
    if(period == 0 && OS_threadNum != 0){
    	if(OS_aperiodic.tam < OS_aperiodic.max_tam){
    		OS_aperiodic.OS_thread[OS_aperiodic.end] = me;
    		OS_aperiodic.end = (OS_aperiodic.end + 1 ) % OS_aperiodic.max_tam;
    		OS_aperiodic.tam++;
    		//OS_readySet |= (1U << (OS_currIdx - 1U));
    	}
    }
    else{
        /* register the thread with the OS */
        OS_thread[OS_threadNum] = me;
        insertion_sort(me);
        OS_threadNum++;
    }
    __enable_irq();
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

/* Adicioando */
void idleThread_main(void) {
    while (1) {
        __WFI();  // Enter low power mode
    }
}
