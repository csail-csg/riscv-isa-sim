// See LICENSE for license details.

#include "htif.h"
#include "sim.h"
#include "encoding.h"
#include <unistd.h>
#include <stdexcept>
#include <stdlib.h>
#include <errno.h>
#include <assert.h>
#include <stddef.h>
#include <poll.h>
#include <stdio.h>
#include <fesvr/term.h>

#if 0
// [sizhuo] comment out original spike HTIF
htif_isasim_t::htif_isasim_t(sim_t* _sim, const std::vector<std::string>& args)
  : htif_pthread_t(args), sim(_sim), reset(true), seqno(1)
{
}

bool htif_isasim_t::tick()
{
  if (done())
    return false;

  do tick_once(); while (reset);

  return true;
}

void htif_isasim_t::tick_once()
{
  packet_header_t hdr;
  recv(&hdr, sizeof(hdr));

  char buf[hdr.get_packet_size()];
  memcpy(buf, &hdr, sizeof(hdr));
  recv(buf + sizeof(hdr), hdr.get_payload_size());
  packet_t p(buf);

  assert(hdr.seqno == seqno);

  switch (hdr.cmd)
  {
    case HTIF_CMD_READ_MEM:
    {
      packet_header_t ack(HTIF_CMD_ACK, seqno, hdr.data_size, 0);
      send(&ack, sizeof(ack));

      uint64_t buf[hdr.data_size];
      for (size_t i = 0; i < hdr.data_size; i++)
        buf[i] = sim->debug_mmu->load_uint64((hdr.addr+i)*HTIF_DATA_ALIGN);
      send(buf, hdr.data_size * sizeof(buf[0]));
      break;
    }
    case HTIF_CMD_WRITE_MEM:
    {
      const uint64_t* buf = (const uint64_t*)p.get_payload();
      for (size_t i = 0; i < hdr.data_size; i++)
        sim->debug_mmu->store_uint64((hdr.addr+i)*HTIF_DATA_ALIGN, buf[i]);

      packet_header_t ack(HTIF_CMD_ACK, seqno, 0, 0);
      send(&ack, sizeof(ack));
      break;
    }
    case HTIF_CMD_READ_CONTROL_REG:
    case HTIF_CMD_WRITE_CONTROL_REG:
    {
      assert(hdr.data_size == 1);
      reg_t coreid = hdr.addr >> 20;
      reg_t regno = hdr.addr & ((1<<20)-1);
      uint64_t old_val, new_val = 0 /* shut up gcc */;

      packet_header_t ack(HTIF_CMD_ACK, seqno, 1, 0);
      send(&ack, sizeof(ack));

      if (coreid == 0xFFFFF) // system control register space
      {
        uint64_t scr = sim->get_scr(regno);
        send(&scr, sizeof(scr));
        break;
      }

      processor_t* proc = sim->get_core(coreid);
      bool write = hdr.cmd == HTIF_CMD_WRITE_CONTROL_REG;
      if (write)
        memcpy(&new_val, p.get_payload(), sizeof(new_val));

      switch (regno)
      {
        case CSR_MTOHOST:
          old_val = proc->get_state()->tohost;
          if (write)
            proc->get_state()->tohost = new_val;
          break;
        case CSR_MFROMHOST:
          old_val = proc->get_state()->fromhost;
          if (write && old_val == 0)
            proc->get_state()->fromhost = new_val;
          break;
        case CSR_MRESET:
          old_val = !proc->running();
          if (write)
          {
            reset = reset & (new_val & 1);
            proc->reset(new_val & 1);
          }
          break;
        default:
          abort();
      }

      send(&old_val, sizeof(old_val));
      break;
    }
    default:
      abort();
  }
  seqno++;
}

bool htif_isasim_t::done()
{
  if (reset)
    return false;
  return !sim->running();
}
#endif

htif_isasim_t::htif_isasim_t(sim_t *_sim, const std::vector<std::string> &args) : 
	htif_riscy_t(args), 
	sim(_sim)
{
	// assertion on 8B alignment & core num
	if(sim->num_cores() != 1) {
		fprintf(stderr, ">> ERROR: spike: should only run 1 core for riscy\n");
		exit(1);
	}
	if(((uint64_t)(sim->mem)) & 0x07ULL) {
		fprintf(stderr, ">> ERROR: spike: memory allocated by is not 8B aligned\n");
		exit(1);
	}

	// set memory
	set_mem((uint64_t*)(sim->mem), sim->memsz);

}

void htif_isasim_t::register_enq_fromhost() {
	// set write fromhost function
	// must be done after processors are created
	// current htif_riscy_t only allows set for core 0
	enq_fromhost.fifo = &(sim->procs[0]->fromhost_fifo);
	set_write_from_host(enq_fromhost);
}

void htif_isasim_t::host_tick(int coreid) {
	processor_t *proc = sim->procs[coreid];
	if(proc->tohost_fifo.empty()) {
		return;
	}

	// assertion: only 1 element in tohost FIFO
	if(proc->tohost_fifo.size() != 1) {
		fprintf(stderr, ">> WARNING: spike: there are %d (>1) elements in tohost FIFO of core %d\n",
				proc->tohost_fifo.size(), coreid);
	}

	// get tohost & handle
	reg_t val = proc->tohost_fifo.front();
  
  // [sizhuo] debug
  //fprintf(stderr, ">> INFO: spike: get mtohost = 0x%016llx\n", (unsigned long long)val);

	proc->tohost_fifo.pop();
	get_to_host(val);
}

void htif_isasim_t::device_tick() {
    device_t *bcd = device_list.get_bcd();
    if(bcd) {
        if(bcd->wait_for_stdin()) {
            int ch = canonical_terminal_t::read();
            if(ch != -1) {
                bcd->feed_stdin(ch);
            }
        }
    } else {
        fprintf(stderr, ">> ERROR: there is no device bcd\n");
    }
}

void htif_isasim_t::target_tick(int coreid) {
	// deq fromhost FIFO to write mfromhost
	processor_t *proc = sim->procs[coreid];
	if(proc->fromhost_fifo.empty()) {
		return;
	}

	// assertion: without keyboard input, there should be only 1 element in fromhost FIFO,
	// and mfromhost should be 0
	if(proc->fromhost_fifo.size() != 1) {
		fprintf(stderr, ">> WARNING: spike: there are %d (>1) elements in fromhost FIFO of core %d\n",
				proc->fromhost_fifo.size(), coreid);
	}
	if(proc->get_csr(CSR_MFROMHOST) != 0) {
		fprintf(stderr, ">> WARNING: spike: mfromhost is not zero when there is response in fromhost FIFO of core %d\n", coreid);
	}

	if(proc->get_csr(CSR_MFROMHOST) == 0) {
		reg_t val = proc->fromhost_fifo.front();
		proc->fromhost_fifo.pop();
		proc->set_csr(CSR_MFROMHOST, val);
    // [sizhuo] debug
    //fprintf(stderr, ">> INFO: spike: set mfromhost = 0x%016llx\n", (unsigned long long)val);
	}
}
