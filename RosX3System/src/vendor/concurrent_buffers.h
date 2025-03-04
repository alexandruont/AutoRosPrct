#ifndef CONCURRENT_BUFFERS_H
#define CONCURRENT_BUFFERS_H

#include <cstdlib>
#include <iostream>
#include <mutex>
#include <atomic>

// SingleProducerSingleConsumerBuffer
template<typename T, size_t S>
class SPSCBuffer {
protected:
	T buffer[S];
	std::atomic<size_t> head = 0;
	std::atomic<size_t> tail = 0;
public:
	SPSCBuffer() = default;

	// This function will also get the item you poped
	bool pop(T& item) {
		size_t current_tail = tail.load(std::memory_order_relaxed);
		if (current_tail == head.load(std::memory_order_acquire)) {
			// Buffer is empty
			return false;
		}
		item = buffer[current_tail];
		tail.store((current_tail + 1) % S, std::memory_order_release);
		return true;
	}

	bool peek(T& item) {
		size_t current_tail = tail.load(std::memory_order_relaxed);
		if (current_tail == head.load(std::memory_order_acquire)) {
			// Buffer is empty
			return false;
		}
		item = buffer[current_tail];
		return true;
	}

	bool push(T& item) {
		size_t current_head = head.load(std::memory_order_relaxed);
		size_t next_head = (current_head + 1) % S;
		if (next_head == tail.load(std::memory_order_acquire)) {
			// Buffer is full
			return false;
		}
		buffer[current_head] = item;
		head.store(next_head, std::memory_order_release);
		return true;
	}
};
// SingleProducerMultiReaderBuffer
template<typename T, size_t S>
class SPMRBuffer {
protected:
	T buffer[S];
	std::atomic<size_t> head = 0;
	std::atomic<size_t> tail = 0;
public:

	// Iterator class definition
	class Iterator {
	public:
		Iterator(const SPMRBuffer<T,S>& buf, size_t pos)
			: syncBuff(buf), position(pos) {}

		// Dereference operator
		const T& operator*() const {
			return syncBuff.buffer[position];
		}

		// Increment operator
		Iterator& operator++() {
			position = (position + 1) % S;
			return *this;
		}

		// Equality operator
		bool operator==(const Iterator& other) const {
			return position == other.position;
		}

		// Inequality operator
		bool operator!=(const Iterator& other) const {
			return !(*this == other);
		}

		size_t getIndex() { return position; }

	private:
		size_t position; // Current position in the buffer
		const SPMRBuffer<T,S>& syncBuff;
	};
	SPMRBuffer() = default;

	bool push(const T& item) {
		size_t current_head = head.load(std::memory_order_relaxed);
		size_t next_head = (current_head + 1) % S;
		if (next_head == tail.load(std::memory_order_acquire)) {
			return false; // Buffer full
		}
		buffer[current_head] = item;
		head.store(next_head, std::memory_order_release);
		return true;
	}

	bool pop(size_t index) {
		size_t current_tail = tail.load(std::memory_order_acquire);
		size_t current_head = head.load(std::memory_order_acquire);
		if (current_tail == current_head){
			return false;
		}
		buffer[index] = buffer[current_tail];
		current_tail = (current_tail + 1) % S;
		tail.store(current_tail);
		return true;
	}

	// Begin iterator (from the tail)
	Iterator begin() const {
		return Iterator(*this, tail.load(std::memory_order_acquire));
	}

	// End iterator (after the head)
	Iterator end() const {
		return Iterator(*this, head.load(std::memory_order_acquire));
	}

	size_t getIteratorPosition(Iterator& pt) { return pt.getIndex(); }

	T* getBuffer() { return buffer; }
};

#endif // !CONCURRENT_BUFFERS_H