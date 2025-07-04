#include "semaforo.h"

void Semaforo::init(int init, int max) {
  count = init;
  this->max = max;
}

void Semaforo::wait() {
  lock(mx);
  rtos::OS_enterCriticalSection();
  while (count <= 0) {
    // Espera ocupada ou pode suspender a thread se desejar
  }
  count--;
  unlock(mx);
}

void Semaforo::signal() {
  lock(mx);
  if (count < max) {
    count++;
  }
  rtos::OS_exitCriticalSection();          // libera seção crítica após liberar o recurso
  unlock(mx);
}
