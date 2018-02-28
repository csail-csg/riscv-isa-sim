#include "devices.h"
#include "sim.h"
#include "fesvr/elfloader.h"

#define MEM_ADDR_BASE 0x0
#define BUSY_BASE 0x8

mem_loader_t::mem_loader_t(sim_t *sim, const std::string &elf_)
  : mem(sim), elf(elf_), mem_addr(0), busy(0), spins_on_busy(0)
{
}

bool mem_loader_t::load(reg_t addr, size_t len, uint8_t* bytes)
{
  fprintf(stderr, "[mem_loader] load addr %lld size %d\n",
          (long long unsigned)addr, int(len));
  if(addr != BUSY_BASE) {
    fprintf(stderr, "[mem_loader] debug: only busy is readable\n");
    return false;
  }
  if(len > sizeof(busy)) {
    fprintf(stderr, "[mem_loader] debug: read len to large");
    return false;
  }
  memcpy(bytes, (char*)(&busy), len);
  
  if (busy) {
    spins_on_busy--;
    if (spins_on_busy <= 0) busy = 0;
  }
  return true;
}

bool mem_loader_t::store(reg_t addr, size_t len, const uint8_t* bytes)
{
  fprintf(stderr, "[mem_loader] store addr %lld size %d\n",
          (long long unsigned)addr, int(len));
  if(addr != MEM_ADDR_BASE) {
    fprintf(stderr, "[mem_loader] debug: only mem_addr is writable\n");
    return false;
  }
  if(len > sizeof(mem_addr)) {
    fprintf(stderr, "[mem_loader] debug: write len to large");
    return false;
  }
  memcpy((char*)(&mem_addr), bytes, len);

  // make sure the entry point matches mem_addr
  reg_t entry;
  dummy_memif_t dummy_mem;
  load_elf(elf.c_str(), &dummy_mem, &entry);
  if(entry != mem_addr) {
    throw std::runtime_error("[mem_loader] mem_addr != elf entry");
    return false;
  }

  // do the real copy
  load_elf(elf.c_str(), &mem, &entry);

  // set busy and how many spins on busy to wait
  busy = 1;
  spins_on_busy = 5;

  return true;
}

