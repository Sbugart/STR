#include "atomic_queue.hpp"

Mutex::Mutex() : locked(false) {}

void Mutex::wait_and_lock()
{
	while(locked){};
	locked = true;
}

void Mutex::unlock()
{
	locked = false;
}

LockGuard::LockGuard(Mutex& _mutex) : mutex(_mutex)
{
	mutex.wait_and_lock();
}

LockGuard::~LockGuard()
{
	mutex.unlock();
}

AtomicQueue128::AtomicQueue128() 
{
	size = 0;
	head = 0;
}

int AtomicQueue128::read(int& out)
{
	LockGuard lock(mutex);
	if(size == 0) return -1;

	out = buffer[head];
	head = (head+1)%BUFFER_CAPACITY;
	size--;

	return 0;
}

int AtomicQueue128::write(int in)
{
	LockGuard lock(mutex);
	if(size == BUFFER_CAPACITY) return -1;

	buffer[(head+size)%BUFFER_CAPACITY] = in;
	size++;
	return 0;
}
