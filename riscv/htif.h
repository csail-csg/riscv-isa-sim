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
	void host_tick(int coreid);
    // [sizhuo] feed bcd with stdin if it needs & there is stdin
	void device_tick(); 
	void target_tick(int coreid);

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
};

#endif
