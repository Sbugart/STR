/*
 * semaforo.h
 *
 *  Created on: May 14, 2025
 *      Author: artur
 */

#ifndef INC_SEMAFORO_H_
#define INC_SEMAFORO_H_

#include "mutex.h"
#include "miros.h"

struct Semaforo {
  int count;
  int max;
  mutex mx;

  void init(int init, int max);
  void wait();
  void signal();
};

#endif /* INC_SEMAFORO_H_ */
