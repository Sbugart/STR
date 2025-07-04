/*
 * miros.h
 *
 *  Created on: Feb 6, 2025
 *      Author: guilh
 */

#ifndef INC_MIROS_H_
#define INC_MIROS_H_
#include <queue>

namespace rtos {
/* Thread Control Block (TCB) */
typedef struct {
    void *sp; /* stack pointer */
    void *sp_base;
    uint32_t timer;
    /* ... other attributes associated with a thread */
    uint32_t period;
    uint32_t release;
    uint32_t total_time;
    uint32_t exec_time;
    uint32_t handler;

    bool is_server;

}OSThread;

typedef struct {
	int32_t init = 0;       // Índice de início da fila circular de threads aperiódicas
	int32_t end = 0;        // Índice de fim da fila circular de threads aperiódicas
	int32_t max_tam = 32;   // Capacidade máxima da fila de threads aperiódicas
	int32_t tam = 0;        // Número atual de threads na fila

	int32_t capacity;       // Capacidade atual disponível no servidor (tempo restante de execução)
	int32_t period;         // Período do servidor (tempo após o qual a capacidade é reabastecida)

	uint32_t time_init;     // Timestamp (tick) do início da última execução do servidor
	uint32_t time_end;      // Timestamp (tick) do fim da última execução do servidor

	// Vetores circulares que controlam o reabastecimento da capacidade:
	// Cada entrada representa um instante futuro em que a capacidade será restaurada.
	uint32_t timers[32];     // Timestamps em que a capacidade será reabastecida
	uint32_t capacitys[32];  // Quantidade de capacidade que será reabastecida nos tempos correspondentes

	uint32_t timer_init = 0; // Índice de início da fila circular de reabastecimento
	uint32_t timer_end = 0;  // Índice de fim da fila circular de reabastecimento

	// Fila circular de threads aperiódicas pendentes a serem executadas pelo servidor
	OSThread *OS_thread[32];

} OSAperiodicThread;


const uint16_t TICKS_PER_SEC = 100U;

extern volatile int32_t OS_criticalNesting;

typedef void (*OSThreadHandler)();

void OS_init(void *stkSto, uint32_t stkSize, uint32_t period, uint32_t capacity, void *stkApe, uint32_t stkApeSize);

/* callback to handle the idle condition */
void OS_onIdle(void);

void main_aperiodicThread(void);

void OS_enterCriticalSection(void);

void OS_exitCriticalSection(void);

void initTimer();


/* this function must be called with interrupts DISABLED */
/**
 * @brief Realiza o escalonamento das tarefas do sistema.
 *
 * Esta função realiza o escalonamento do sistema por meio do Rate Monotonic, onde
 * a próxima tarefa a ser executada é selecionada com base no período.
 * O escalonador considera as tarefas com maior prioridade as que apresentam um
 * menor período, que seriam as que possuem um menor índice no vetor de tarefas.
 *
 * Se uma nova tarefa de maior prioridade estiver pronta, a função aciona uma troca
 * de contexto para garantir preempção imediata.
 *
 * Essa função é normalmente chamada após o tratamento do SysTick (timer do sistema).
 *
 * @note A tarefa idle (de menor prioridade) é executada caso nenhuma outra tarefa esteja pronta.
 */

void OS_sched(void);

/* transfer control to the RTOS to run the threads */
void OS_run(void);

/* blocking delay */
void OS_delay(uint32_t ticks);

/* process all timeouts */
void OS_tick(void);

/* callback to configure and start interrupts */
void OS_onStartup(void);

void thread_wrapper(void);
void aperiodic_thread_wrapper(void);

/*	Implementação do insertion sort para o vetor de threads*/
void insertion_sort(std::vector<OSThread*>& vector, OSThread *me);

void OSThread_start(
    OSThread *me,
    OSThreadHandler threadHandler,
    void *stkSto, uint32_t stkSize,
	uint32_t period, uint32_t release,
    uint32_t total_time,
	bool is_server);

}

#endif /* INC_MIROS_H_ */
