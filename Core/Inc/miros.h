/*
 * miros.h
 *
 *  Created on: Feb 6, 2025
 *      Author: guilh
 */

#ifndef INC_MIROS_H_
#define INC_MIROS_H_

namespace rtos {


class LockGuard {
public:
    LockGuard();
    ~LockGuard();
};


const unsigned int BUFFER_CAPACITY = 128U;
class AtomicQueue128 {
public:
    AtomicQueue128();
    
    // Retorna 0 em caso de sucesso, -1 se a fila estiver vazia
    int read(int& out);
    
    // Retorna 0 em caso de sucesso, -1 se a fila estiver cheia
    int write(int in);

private:
    uint8_t size;
    uint8_t head;
    int buffer[BUFFER_CAPACITY];
};



/* Thread Control Block (TCB) */
typedef struct {
    void *sp; /* stack pointer */
    uint32_t start_pc; /* start address of the thread */
    uint32_t timeout; /* timeout delay down-counter */
    uint32_t period_ticks; /* period of the thread */
    bool finished_execution;
    /* ... other attributes associated with a thread */
} OSThread;

typedef struct {
    void *sp; /* stack pointer */
    uint32_t start_pc; /* start address of the thread */
    uint32_t period_ticks; /* period of the thread */
    uint32_t max_random_period_deviation;   /* To simulate aperiodic tasks*/
    uint32_t next_arrival_tick;
    /* ... other attributes associated with a thread */
} OSAperiodicThread;

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
    void *stkSto, uint32_t period_ticks, uint32_t stkSize);

void OSAperiodicThread_start(
    OSAperiodicThread *me,
    OSThreadHandler threadHandler,
    void *stkSto, uint32_t period_ticks, uint32_t max_random_period_deviation, uint32_t stkSize);
}
#endif /* INC_MIROS_H_ */
