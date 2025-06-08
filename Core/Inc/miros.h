/*
 * miros.h
 *
 *  Created on: Feb 6, 2025
 *      Author: guilh
 */

#ifndef INC_MIROS_H_
#define INC_MIROS_H_

namespace rtos {
/* Thread Control Block (TCB) */
typedef struct {
    void *sp; /* stack pointer */
    uint32_t start_pc; /* start address of the thread */
    uint32_t timeout; /* timeout delay down-counter */
    uint32_t period_ticks; /* period of the thread */
    bool finished_execution;
    uint32_t max_random_period_deviation;   /* To simulate aperiodic tasks*/
    int32_t current_period_deviation;
    /* ... other attributes associated with a thread */
} OSThread;

const uint16_t TICKS_PER_SEC = 10U;

typedef void (*OSThreadHandler)();

void OS_init(void *stkSto, uint32_t stkSize);

/* callback to handle the idle condition */
void OS_onIdle(void);

/* this function must be called with interrupts DISABLED */
void OS_sched(void);

/* transfer control to the RTOS to run the threads */
void OS_run(void);

/* blocking delay */
void OS_delay(uint32_t ticks);

/* process all timeouts */
void OS_tick(void);

/* callback to configure and start interrupts */
void OS_onStartup(void);

void OSThread_start(
    OSThread *me,
    OSThreadHandler threadHandler,
    void *stkSto, uint32_t period_ticks, uint32_t max_random_period_deviation, uint32_t stkSize);
}

#endif /* INC_MIROS_H_ */
