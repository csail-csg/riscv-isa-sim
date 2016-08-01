// See LICENSE for license details.

#ifndef _HTIF_H
#define _HTIF_H

// [sizhuo] do our own HTIF
//#include <fesvr/htif_pthread.h>
#include "htif_riscy.h"
#include <queue>

class sim_t;
struct packet;

// this class implements the host-target interface for program loading, etc.
// a simpler implementation would implement the high-level interface
// (read/write cr, read/write chunk) directly, but we implement the lower-
// level serialized interface to be more similar to real target machines.

#if 0
// [sizhuo] comment out original spike HTIF
class htif_isasim_t : public htif_pthread_t
{
public:
  htif_isasim_t(sim_t* _sim, const std::vector<std::string>& args);
  bool tick();
  bool done();

private:
  sim_t* sim;
  bool reset;
  uint8_t seqno;

  void tick_once();
};
#endif

class htif_isasim_t : public htif_riscy_t {
public:
	htif_isasim_t(sim_t *_sim, const std::vector<std::string>& args);
	~htif_isasim_t() {}
	void register_enq_fromhost();
	// void host_tick(int coreid); // removed the fifo this dequeued from in favor of calling get_to_host directly
	void check_stdin_for_bcd(); // was device_tick, now it just does what it says
	void check_from_host(int coreid); // was target_tick, now it just checks the fromhost fifo and updates mfromhost if necessary
	void disable_stdout();

private:
	sim_t *sim;

	// function to enq from host
	struct enq_fifo_func_t {
		std::queue<reg_t> *fifo = NULL;
		void operator()(reg_t x) {
			fifo->push(x);
		}
	};
	enq_fifo_func_t enq_fromhost;

    // memory
    // function to DMA read mem
    struct read_mem_t {
        uint64_t *mem;
        void operator()(addr_t addr, size_t len, void *dst) {
            memcpy(dst, &(mem[addr / sizeof(uint64_t)]), len);
        }
    };
    read_mem_t read_mem;
    // function to DMA write mem
    struct write_mem_t {
        uint64_t *mem;
        void operator()(addr_t addr, size_t len, const void *src) {
            memcpy(&(mem[addr / sizeof(uint64_t)]), src, len);
        }
    };
    write_mem_t write_mem;
};

#endif
