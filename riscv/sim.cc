// See LICENSE for license details.

#include "sim.h"
#include "htif.h"
#include <map>
#include <iostream>
#include <climits>
#include <cstdlib>
#include <cassert>
#include <signal.h>

volatile bool ctrlc_pressed = false;
static void handle_signal(int sig)
{
  if (ctrlc_pressed)
    exit(-1);
  ctrlc_pressed = true;
  signal(sig, &handle_signal);
}

sim_t::sim_t(const char* isa, size_t nprocs, size_t mem_mb,
             const std::vector<std::string>& args)
  : //htif(new htif_isasim_t(this, args)), // [sizhuo] create HTIF later 
	procs(std::max(nprocs, size_t(1))),
    rtc(0), current_step(0), current_proc(0), debug(false)
{
  //signal(SIGINT, &handle_signal); // register this later

  // allocate target machine's memory, shrinking it as necessary
  // until the allocation succeeds
  size_t memsz0 = (size_t)mem_mb << 20;
  size_t quantum = 1L << 20;
  if (memsz0 == 0)
    memsz0 = 1L << (sizeof(size_t) == 8 ? 32 : 30);

  memsz = memsz0;
  while ((mem = (char*)calloc(1, memsz)) == NULL)
    memsz = memsz*10/11/quantum*quantum;

  if (memsz != memsz0)
    fprintf(stderr, "warning: only got %lu bytes of target mem (wanted %lu)\n",
            (unsigned long)memsz, (unsigned long)memsz0);

  debug_mmu = new mmu_t(mem, memsz);

  // [sizhuo] create HTIF
  htif.reset(new htif_isasim_t(this, args));

  for (size_t i = 0; i < procs.size(); i++) {
    procs[i] = new processor_t(isa, this, i);
	// [sizhuo] manually reset processors (we don't reset by waiting for write on MRESET)
	procs[i]->reset(false);
  }

  // [sizhuo] register enq fromhost FIFOs (must be done after procs created)
  // and start HTIF by loading programs etc.
  htif->register_enq_fromhost();
  htif->start();

  fprintf(stderr, ">> INFO: spike: HTIF started\n");

  // [sizhuo] register handler after htif is created, 
  // override handler registered by fesvr/htif_t
  signal(SIGINT, &handle_signal);
}

sim_t::~sim_t()
{
  for (size_t i = 0; i < procs.size(); i++)
    delete procs[i];
  delete debug_mmu;
  free(mem);
}

void sim_t::send_ipi(reg_t who)
{
  if (who < procs.size())
    procs[who]->deliver_ipi();
}

reg_t sim_t::get_scr(int which)
{
  switch (which)
  {
    case 0: return procs.size();
    case 1: return memsz >> 20;
    default: return -1;
  }
}

int sim_t::run()
{
  if (!debug && log)
    set_procs_debug(true);

  while (!htif->done()) //(htif->tick()) // [sizhuo] use done() instead
  {
    if (debug || ctrlc_pressed)
      interactive();
    else
      step(INTERLEAVE);
  }
  return htif->exit_code();
}

void sim_t::step(size_t n)
{
  /*
  for (size_t i = 0, steps = 0; i < n; i += steps)
  {
    steps = std::min(n - i, INTERLEAVE - current_step);
    procs[current_proc]->step(steps);

    current_step += steps;
    if (current_step == INTERLEAVE)
    {
      current_step = 0;
      // procs[current_proc]->yield_load_reservation();
      if (++current_proc == procs.size()) {
        current_proc = 0;
        rtc += INTERLEAVE / INSNS_PER_RTC_TICK;
      }

      htif->tick();
    }
  }
  */
  // [sizhuo] use a simpler way, tick HTIF after every step
  for(size_t i = 0; i < n; i++) {
    procs[current_proc]->step(1);
    htif->host_tick(current_proc);
	htif->device_tick(); // [sizhuo] comment out if we don't want stdin
    htif->target_tick(current_proc);

    current_step++;
    if (current_step == INTERLEAVE) {
      current_step = 0;
      if (++current_proc == procs.size()) {
        current_proc = 0;
        rtc += INTERLEAVE / INSNS_PER_RTC_TICK;
      }
	}
  }
}

bool sim_t::running()
{
  for (size_t i = 0; i < procs.size(); i++)
    if (procs[i]->running())
      return true;
  return false;
}

void sim_t::stop()
{
  //procs[0]->state.tohost = 1;
  //while (htif->tick())
  //  ;
  // [sizhuo] this function is actually never called
  procs[0]->set_csr(CSR_MTOHOST, 1);
}

void sim_t::set_debug(bool value)
{
  debug = value;
}

void sim_t::set_log(bool value)
{
  log = value;
}

void sim_t::set_histogram(bool value)
{
  histogram_enabled = value;
  for (size_t i = 0; i < procs.size(); i++) {
    procs[i]->set_histogram(histogram_enabled);
  }
}

void sim_t::set_procs_debug(bool value)
{
  for (size_t i=0; i< procs.size(); i++)
    procs[i]->set_debug(value);
}

bool sim_t::mmio_load(reg_t addr, size_t len, uint8_t* bytes)
{
  return false;
}

bool sim_t::mmio_store(reg_t addr, size_t len, const uint8_t* bytes)
{
  return false;
}
