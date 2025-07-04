#include "mutex.h"

void lock(mutex& locked) {
    while (locked.mux.exchange(true, std::memory_order_acquire)) {
        // Espera ativa
    }
}

void unlock(mutex& locked) {
    locked.mux.store(false, std::memory_order_release);
}
