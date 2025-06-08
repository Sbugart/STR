#ifndef ATOMIC_QUEUE_HPP_
#define ATOMIC_QUEUE_HPP_

#include <atomic>
#include <cstdint>

class Mutex {
public:
    Mutex();

private:
    friend class LockGuard;
    void wait_and_lock();
    void unlock();
    std::atomic<bool> locked;
};

class LockGuard {
public:
    LockGuard(Mutex& _mutex);
    ~LockGuard();

private:
    Mutex& mutex;
};

#define BUFFER_CAPACITY 128

class AtomicQueue128 {
public:
    AtomicQueue128();
    
    // Retorna 0 em caso de sucesso, -1 se a fila estiver vazia
    int read(int& out);
    
    // Retorna 0 em caso de sucesso, -1 se a fila estiver cheia
    int write(int in);

private:
    Mutex mutex;
    uint8_t size;
    uint8_t head;
    int buffer[BUFFER_CAPACITY];
};

#endif // ATOMIC_QUEUE_HPP_