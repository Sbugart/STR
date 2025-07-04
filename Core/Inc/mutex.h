#ifndef MUTEX_H
#define MUTEX_H

#include <atomic>

struct mutex {
    std::atomic<bool> mux = false;
};

void lock(mutex& locked);

void unlock(mutex& locked);

#endif
