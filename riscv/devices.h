#ifndef _RISCV_DEVICES_H
#define _RISCV_DEVICES_H

#include "decode.h"
#include "fesvr/memif.h"
#include <cstdlib>
#include <string>
#include <map>
#include <vector>

class processor_t;

class abstract_device_t {
 public:
  virtual bool load(reg_t addr, size_t len, uint8_t* bytes) = 0;
  virtual bool store(reg_t addr, size_t len, const uint8_t* bytes) = 0;
  virtual ~abstract_device_t() {}
};

// This is a mapping from (negative)base addr to device ptr. Load/Store to bus
// will be redirected to the right device.
class bus_t : public abstract_device_t {
 public:
  bool load(reg_t addr, size_t len, uint8_t* bytes);
  bool store(reg_t addr, size_t len, const uint8_t* bytes);
  void add_device(reg_t addr, abstract_device_t* dev);

  std::pair<reg_t, abstract_device_t*> find_device(reg_t addr);

 private:
  std::map<reg_t, abstract_device_t*> devices;
};

class rom_device_t : public abstract_device_t {
 public:
  rom_device_t(std::vector<char> data);
  bool load(reg_t addr, size_t len, uint8_t* bytes);
  bool store(reg_t addr, size_t len, const uint8_t* bytes);
  const std::vector<char>& contents() { return data; }
 private:
  std::vector<char> data;
};

// DRAM memory. DRAM_BASE is default to be 0x8000_0000 (see spike.cc)
class mem_t : public abstract_device_t {
 public:
  mem_t(size_t size) : len(size) {
    data = (char*)calloc(1, size);
    if (!data)
      throw std::runtime_error("couldn't allocate " + std::to_string(size) + " bytes of target memory");
  }
  mem_t(const mem_t& that) = delete;
  ~mem_t() { free(data); }

  bool load(reg_t addr, size_t len, uint8_t* bytes) { return false; }
  bool store(reg_t addr, size_t len, const uint8_t* bytes) { return false; }
  char* contents() { return data; }
  size_t size() { return len; }

 private:
  char* data;
  size_t len;
};

// timer + inter-proc SW interrupt
class clint_t : public abstract_device_t {
 public:
  clint_t(std::vector<processor_t*>&);
  bool load(reg_t addr, size_t len, uint8_t* bytes);
  bool store(reg_t addr, size_t len, const uint8_t* bytes);
  size_t size() { return CLINT_SIZE; }
  void increment(reg_t inc);
 private:
  // when accessing mtime and mtimecmp with MMIO, these regs are 64-bit wide.
  typedef uint64_t mtime_t;
  typedef uint64_t mtimecmp_t;
  // when accessing msip (of each processor) with MMIO, these regs are 32-bit
  // wide
  typedef uint32_t msip_t;
  std::vector<processor_t*>& procs;
  mtime_t mtime;
  std::vector<mtimecmp_t> mtimecmp;
};

class sim_t;
class mem_loader_t : public abstract_device_t {
 public:
  mem_loader_t(sim_t *sim, const std::string &elf);
  virtual bool load(reg_t addr, size_t len, uint8_t* bytes);
  virtual bool store(reg_t addr, size_t len, const uint8_t* bytes);
  virtual ~mem_loader_t() {}

 private:
  memif_t mem;
  std::string elf;
  // MMIO accessible regs
  reg_t mem_addr; // target mem addr to start loading elf
  reg_t busy; // whether loader is still working
  // after loading memory, we wait for certain spins on busy before we reset
  // busy bit
  int spins_on_busy;
};

#endif
