#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <iostream>
#include <algorithm>

template <typename T>
class RingBuffer {
    private:
        // size of the buffer
        uint16_t size;

        // maximum size of buffer
        uint16_t max_size;

        // real index of the ring buffer head
        int16_t head;

        // real index of the ring buffer tail
        int16_t tail;

        // buffer to store data
        std::vector<T> buffer;

    public:
        // constructor
        RingBuffer(uint16_t max_size)
        {
            // allocate memory
            buffer.resize(max_size);

            // set buffer size to zero
            size = 0;

            // set maximum size
            this->max_size = max_size;

            // set head and tail to zero
            head = 0;
            tail = 0;
        }

        // deconstructor
        ~RingBuffer() {}

        // method for getting the size of the rung buffer
        uint16_t getSize()
        {
            return size;
        }

        // method for pushing the data at the front of the ring buffer
        void push(T const data)
        {
            buffer[head] = data;
            
            // increment the header in circular manner
            head++;
            if(head >= max_size) head = 0;

            // increment size
            size = std::min((uint16_t)(size+1), max_size);

            // setup the tail index 
            if(size == max_size)
            {
                tail = head - max_size;
                if(tail < 0) tail += max_size;
            }
        }

        // method for popping the data at the front of the ring buffer
        T pop()
        {
            // make sure atleast one data exists
            if(size == 0)
            {
                std::cerr << "no data available to pop" << std::endl;
                return T();
            }
            else
            {
                // decrement the head and return the data pointed by head
                head--;
                if(head < 0) head+= max_size;
                size--;
                return buffer[head];
            }
        }

        // method for getting data at a certain index
        T get(uint16_t index)
        {
            // make sure that data requested at index exists
            if(index >= size)
            {
                std::cerr << "index out of range" << std::endl;
                return T();
            }
            else
            {
                // get the actual index and return data
                int16_t idx = tail + index;
                if(idx >= max_size)
                {
                    idx = idx - max_size;
                }
                return buffer[idx];
            }
        }

        // method for getting data pointer at a certain index
        T * get_ptr(uint16_t index)
        {
            // make sure that data requested at index exists
            if(index >= size)
            {
                std::cerr << "index out of range" << std::endl;
                return (T *)NULL;
            }
            else
            {
                // get the actual index and return data pointer
                int16_t idx = tail + index;
                if(idx >= max_size)
                {
                    idx = idx - max_size;
                }
                return &buffer[idx];
            }
        }
};

#endif