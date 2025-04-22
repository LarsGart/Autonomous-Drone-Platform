#include "RingBuffer.h"

RingBuffer::RingBuffer() {
  for (size_t i = 0; i < size; i++) {
    buffer[i] = 0; // Initialize buffer to zero
  }
  head = 0; // Initialize head index
  tail = 0; // Initialize tail index
  empty = true; // Initialize empty flag
};

void RingBuffer::push(uint8_t data) {
  buffer[tail] = data; // Push data into the ring buffer
  tail = (tail == mask) ? 0 : tail + 1; // Update tail index, wrap around if needed
  empty = false; // Set empty flag to false
  
  // If the buffer is full, overwrite the oldest data by moving head forward
  if (tail == head) {
    head = (head == mask) ? 0 : head + 1;
  }
}

uint8_t RingBuffer::pop() {
  uint8_t data = buffer[head]; // Pop data from the ring buffer
  if (head == tail) {
    empty = true; // Set empty flag to true if buffer is empty
  } else {
    head = (head == mask) ? 0 : head + 1; // Update head index, wrap around if needed
  }
  return data; // Return the popped data
}

bool RingBuffer::isEmpty() {
  return empty; // Return the empty flag
}