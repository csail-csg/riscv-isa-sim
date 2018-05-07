#pragma once
#include <semaphore.h>
#include <vector>

// double buffering that transfers chuncks of data from one producer to one
// consumer

template<class T>
class sync_buffer_t {
public:
    sync_buffer_t(int size) : buff_size(size) {
        buffers[0].buffer.resize(size);
        buffers[1].buffer.resize(size);
        // initially, producer owns buffer 0, so it can directly produce;
        // consumer owns buffer 1, it will immediately signal consume done on
        // buffer 1, and switch to buffer 0 to wait for produce done
        produce_buffer = &(buffers[0]);
        consume_buffer = &(buffers[1]);
    }

    void produce(const T& x) {
        // we must be able to produce
        int idx = produce_buffer->produce_idx;
        produce_buffer->buffer[idx] = x;
        idx++;
        produce_buffer->produce_idx = idx;
        // check if we need to switch buffer
        if(idx >= buff_size) {
            sem_post(&(produce_buffer->produce_done));
            produce_buffer = produce_buffer == buffers ? &(buffers[1]) : buffers;
            sem_wait(&(produce_buffer->consume_done));
            produce_buffer->produce_idx = 0; // init idx
        }
    }

    // XXX the returned referece is guaranteed to be valid before the next call
    // to consume
    T& consume() {
        int idx = consume_buffer->consume_idx;
        // check if we need to switch buffer
        if(idx >= consume_buffer->produce_idx) {
            sem_post(&(consume_buffer->consume_done));
            consume_buffer = consume_buffer == buffers ? &(buffers[1]) : buffers;
            sem_wait(&(consume_buffer->produce_done));
            idx = 0; // init idx
        }
        // get data
        consume_buffer->consume_idx = idx + 1;
        return consume_buffer->buffer[idx];
    }

private:
    struct single_buffer_t {
        single_buffer_t() : produce_idx(0), consume_idx(0) {
            sem_init(&produce_done, 0, 0);
            sem_init(&consume_done, 0, 0);
        }
        sem_t produce_done;
        sem_t consume_done;
        std::vector<T> buffer;
        int produce_idx; // idx to put the entry
        int consume_idx; // idx to get the entry
    } __attribute__ ((aligned (64)));

    const int buff_size;
    single_buffer_t buffers[2];
    single_buffer_t *produce_buffer __attribute__ ((aligned (64)));
    single_buffer_t *consume_buffer __attribute__ ((aligned (64)));
};
