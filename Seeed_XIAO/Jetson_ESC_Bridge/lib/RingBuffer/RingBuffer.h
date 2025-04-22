#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <Arduino.h>

class RingBuffer {
  public:
    // Constructor
    RingBuffer();

    // Methods
    void push(uint8_t data); // Push data into the ring buffer
    uint8_t pop(); // Pop data from the ring buffer
    bool isEmpty(); // Check if the ring buffer is empty

  private:
    static const size_t size = 32; // Size of the ring buffer
    static const size_t mask = size - 1; // Mask for wrapping around the buffer
    uint8_t buffer[size]; // Buffer array
    size_t head; // Head index
    size_t tail; // Tail index
    bool empty; // Flag to indicate if the buffer is empty
};

#endif