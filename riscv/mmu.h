// See LICENSE for license details.

#ifndef _RISCV_MMU_H
#define _RISCV_MMU_H

#include "decode.h"
#include "trap.h"
#include "common.h"
#include "config.h"
#include "sim.h"
#include "processor.h"
#include "memtracer.h"
#include <stdlib.h>
#include <assert.h>
#include <vector>
#include <stdio.h>

// virtual memory configuration
#define PGSHIFT 12
const reg_t PGSIZE = 1 << PGSHIFT;
const reg_t PGMASK = ~(PGSIZE-1);

struct insn_fetch_t
{
  insn_func_t func;
  insn_t insn;
};

struct icache_entry_t {
  reg_t tag;
  reg_t pad;
  insn_fetch_t data;
};

struct tlb_entry_t {
  char* host_offset;
  reg_t target_offset;
};

class trigger_matched_t
{
  public:
    trigger_matched_t(int index,
        trigger_operation_t operation, reg_t address, reg_t data) :
      index(index), operation(operation), address(address), data(data) {}

    int index;
    trigger_operation_t operation;
    reg_t address;
    reg_t data;
};

// A self-invalidate write-through D$. For simplicity, cache is direct-mapped.
// This could allow reordering of dependent loads. We directly use the host
// addr (i.e., pointer value in mem_t) as cache address.
//
// We use write-through to avoid issues related to coherece and page walk.
class siwt_cache_t {
public:
  siwt_cache_t(sim_t* s, processor_t* p) :
    sim(s),
    proc(p),
    // disable D$ for debug MMU which is used for HTIF, etc.
    enabled(sim->dcache_enabled && proc != NULL),
    max_hits(sim->dcache_max_hits),
    data_ram(NULL),
    valid_ram(NULL),
    tag_ram(NULL),
    hit_ram(NULL),
    tohost(NULL),
    fromhost(NULL)
  {
    data_ram = (char*)calloc(1 << index_width, 1 << log_line_bytes);
    if (!data_ram) {
      throw std::runtime_error("cannot allocate D$ data");
    }
    tag_ram = (addr_t*)calloc(1 << index_width, sizeof(addr_t));
    if (!tag_ram) {
      throw std::runtime_error("cannot allocate D$ tag");
    }
    valid_ram = (bool*)calloc(1 << index_width, sizeof(bool));
    if (!valid_ram) {
      throw std::runtime_error("cannot allocate D$ valid");
    }
    hit_ram = (int*)calloc(1 << index_width, sizeof(int));
    if (!hit_ram) {
      throw std::runtime_error("cannot allocate D$ hit count");
    }

    fprintf(stderr, "info: dcache: enable %d, max hits %d\n",
            int(enabled), max_hits);
  }

  ~siwt_cache_t() {
    free(data_ram);
    free(tag_ram);
    free(valid_ram);
    free(hit_ram);
  }

  inline void load(char* addr, reg_t len, char* dst, bool exclusive, bool reserve) {
    // check no cross line boundary
    assert(get_offset((addr_t)addr) + len <= (1 << log_line_bytes));
    // load reserve should be exclusive and from a real core (instead of DMA)
    if (reserve) {
      assert(exclusive);
      assert(proc != NULL);
    }

    // perform load in case we don't use cache
    if (!enabled || is_htif_addr(addr)) {
      memcpy(dst, addr, len);
      // make reservation
      if (reserve) {
        make_reservation(addr);
      }
      return;
    }

    // cache look up (load for exclusivness, e.g., AMO-load or LR, should fetch
    // from memory)
    addr_t index = get_index((addr_t)addr);
    addr_t tag = get_tag((addr_t)addr);
    if (exclusive || !valid_ram[index] || tag != tag_ram[index]) {
      refill_line(addr, index, tag);
    }
    else {
      hit_ram[index]++;
    }
    memcpy(dst, get_data_ptr((addr_t)addr), len);
    // evict after certain number of hits (negative max_hits means no self
    // eviction forever)
    if (max_hits >= 0 && hit_ram[index] >= max_hits) {
      valid_ram[index] = false;
    }
    // make reservation
    if (reserve) {
      make_reservation(addr);
    }
  }

  inline reg_t store(char* addr, reg_t len, const char* val, bool conditional) {
    // check no cross line boundary
    assert(get_offset((addr_t)addr) + len <= (1 << log_line_bytes));

    // determine store-cond success or fail
    if (conditional) {
      assert(proc);
      if (((addr_t)addr >> 3) != ((addr_t)(proc->get_state()->load_reservation) >> 3)) {
        // store-cond fails, skip the rest of actions, reset reservation
        proc->yield_load_reservation();
        return 1;
      }
      // store-cond succeeds, keep going, reset reservation
      proc->yield_load_reservation();
    }

    // write through
    memcpy(addr, val, len);
    // Invalidate other cores' load reservation on the same double word
    for (processor_t *p : sim->procs) {
      if (p != proc) {
        if (((addr_t)addr >> 3) == ((addr_t)(p->get_state()->load_reservation) >> 3)) {
          p->yield_load_reservation();
        }
      }
    }
    if (!enabled) {
      return 0; // store-cond is performed
    }

    // evict written line to avoid subword problems
    addr_t index = get_index((addr_t)addr);
    addr_t tag = get_tag((addr_t)addr);
    if (valid_ram[index] && tag == tag_ram[index]) {
      valid_ram[index] = false;
    }
    return 0; // store-cond is performed
  }

  inline void flush() {
    if (!enabled) {
      return;
    }
    memset(valid_ram, 0, 1 << index_width);
  }

  void set_htif_addrs() {
    addr_t to = sim->get_tohost_addr();
    addr_t from = sim->get_fromhost_addr();
    tohost = sim->addr_to_mem(to);
    fromhost = sim->addr_to_mem(from);
    fprintf(stderr, "info: mmu: tohost %llx, fromhost %llx\n",
            (long long unsigned)to, (long long unsigned)from);
  }

private:
  sim_t *sim;
  processor_t *proc;

  // 512KB cache, 64B line
  static const int log_line_bytes = 6;
  static const int index_width = 13;

  static const addr_t index_mask = (addr_t(1) << index_width) - 1;
  static const addr_t offset_mask = (addr_t(1) << log_line_bytes) - 1;
  static const addr_t index_offset_mask = (addr_t(1) << (index_width + log_line_bytes)) - 1;

  const bool enabled; // whether the cache is enabled or not

  const int max_hits; // self-invalidate after a certain number of hits

  char *data_ram;
  bool *valid_ram;
  addr_t *tag_ram;
  int *hit_ram;

  char* tohost;
  char* fromhost;

  inline addr_t get_tag(addr_t addr) {
    return addr >> (index_width + log_line_bytes);
  }

  inline addr_t get_index(addr_t addr) {
    return (addr >> log_line_bytes) & index_mask;
  }

  inline addr_t get_offset(addr_t addr) {
    return addr & offset_mask;
  }

  inline char* get_data_ptr(addr_t addr) {
    return data_ram + (addr & index_offset_mask);
  }

  inline void refill_line(char* addr, addr_t index, addr_t tag) {
    addr_t offset = get_offset((addr_t)addr);
    memcpy(&data_ram[index << log_line_bytes], addr - offset, 1 << log_line_bytes);
    valid_ram[index] = true;
    tag_ram[index] = tag;
    hit_ram[index] = 0;
  }

  inline bool is_htif_addr(char* addr) {
    return addr == tohost || addr == fromhost;
  }

  void inline make_reservation(char* addr) {
    proc->get_state()->load_reservation = addr;
  }
};

// this class implements a processor's port into the virtual memory system.
// an MMU and instruction cache are maintained for simulator performance.
class mmu_t
{
public:
  mmu_t(sim_t* sim, processor_t* proc);
  ~mmu_t();

  inline reg_t misaligned_load(reg_t addr, size_t size)
  {
#ifdef RISCV_ENABLE_MISALIGNED
    // our hardware doesn't support misaligned access
    static_assert(false);
    reg_t res = 0;
    for (size_t i = 0; i < size; i++)
      res += (reg_t)load_uint8(addr + i) << (i * 8);
    return res;
#else
    throw trap_load_address_misaligned(addr);
#endif
  }

  inline void misaligned_store(reg_t addr, reg_t data, size_t size)
  {
#ifdef RISCV_ENABLE_MISALIGNED
    // our hardware doesn't support misaligned access
    static_assert(false);
    for (size_t i = 0; i < size; i++)
      store_uint8(addr + i, data >> (i * 8));
#else
    throw trap_store_address_misaligned(addr);
#endif
  }

  // template for functions that load an aligned value from memory
  // 2nd arg is used for loads for exclusivness, e.g., AMO-load or LR
  // 3rd arg is whether we reserve the addr, just for LR
  #define load_func(type) \
    inline type##_t load_##type(reg_t addr, bool exclusive = false, bool reserve = false) { \
      if (unlikely(addr & (sizeof(type##_t)-1))) { \
        assert(!exclusive); \
        assert(!reserve); \
        return misaligned_load(addr, sizeof(type##_t)); \
      } \
      reg_t vpn = addr >> PGSHIFT; \
      if (likely(tlb_load_tag[vpn % TLB_ENTRIES] == vpn)) { \
        type##_t data = 0; \
        dcache.load(tlb_data[vpn % TLB_ENTRIES].host_offset + addr, sizeof(type##_t), (char*)&data, exclusive, reserve); \
        return data; \
      } \
      if (unlikely(tlb_load_tag[vpn % TLB_ENTRIES] == (vpn | TLB_CHECK_TRIGGERS))) { \
        type##_t data = 0; \
        dcache.load(tlb_data[vpn % TLB_ENTRIES].host_offset + addr, sizeof(type##_t), (char*)&data, exclusive, reserve); \
        if (!matched_trigger) { \
          matched_trigger = trigger_exception(OPERATION_LOAD, addr, data); \
          if (matched_trigger) \
            throw *matched_trigger; \
        } \
        return data; \
      } \
      type##_t res; \
      load_slow_path(addr, sizeof(type##_t), (uint8_t*)&res, exclusive, reserve); \
      return res; \
    }

  // load value from memory at aligned address; zero extend to register width
  load_func(uint8)
  load_func(uint16)
  load_func(uint32)
  load_func(uint64)

  // load value from memory at aligned address; sign extend to register width
  load_func(int8)
  load_func(int16)
  load_func(int32)
  load_func(int64)

  // template for functions that store an aligned value to memory
  // 3rd arg is just for store-cond
  // return value is just for store-cond. return 0 if store-cond succeeds,
  // return 1 if store-cond fails
  #define store_func(type) \
    reg_t store_##type(reg_t addr, type##_t val, bool conditional = false) { \
      if (unlikely(addr & (sizeof(type##_t)-1))) { \
        assert(!conditional); \
        misaligned_store(addr, val, sizeof(type##_t)); \
        return 0; \
      } \
      reg_t vpn = addr >> PGSHIFT; \
      if (likely(tlb_store_tag[vpn % TLB_ENTRIES] == vpn)) \
        return dcache.store(tlb_data[vpn % TLB_ENTRIES].host_offset + addr, sizeof(type##_t), (const char*)&val, conditional); \
      else if (unlikely(tlb_store_tag[vpn % TLB_ENTRIES] == (vpn | TLB_CHECK_TRIGGERS))) { \
        if (!matched_trigger) { \
          matched_trigger = trigger_exception(OPERATION_STORE, addr, val); \
          if (matched_trigger) \
            throw *matched_trigger; \
        } \
        return dcache.store(tlb_data[vpn % TLB_ENTRIES].host_offset + addr, sizeof(type##_t), (const char*)&val, conditional); \
      } \
      else \
        return store_slow_path(addr, sizeof(type##_t), (const uint8_t*)&val, conditional); \
    }

  // template for functions that perform an atomic memory operation
  #define amo_func(type) \
    template<typename op> \
    type##_t amo_##type(reg_t addr, op f) { \
      if (addr & (sizeof(type##_t)-1)) \
        throw trap_store_address_misaligned(addr); \
      try { \
        auto lhs = load_##type(addr, true, false); \
        store_##type(addr, f(lhs), false); \
        return lhs; \
      } catch (trap_load_page_fault& t) { \
        /* AMO faults should be reported as store faults */ \
        throw trap_store_page_fault(t.get_badaddr()); \
      } catch (trap_load_access_fault& t) { \
        /* AMO faults should be reported as store faults */ \
        throw trap_store_access_fault(t.get_badaddr()); \
      } \
    }

  // store value to memory at aligned address
  store_func(uint8)
  store_func(uint16)
  store_func(uint32)
  store_func(uint64)

  // perform an atomic memory operation at an aligned address
  amo_func(uint32)
  amo_func(uint64)

  static const reg_t ICACHE_ENTRIES = 1024;

  inline size_t icache_index(reg_t addr)
  {
    return (addr / PC_ALIGN) % ICACHE_ENTRIES;
  }

  inline icache_entry_t* refill_icache(reg_t addr, icache_entry_t* entry)
  {
    auto tlb_entry = translate_insn_addr(addr);
    insn_bits_t insn = *(uint16_t*)(tlb_entry.host_offset + addr);
    int length = insn_length(insn);

    if (likely(length == 4)) {
      insn |= (insn_bits_t)*(const int16_t*)translate_insn_addr_to_host(addr + 2) << 16;
    } else if (length == 2) {
      insn = (int16_t)insn;
    } else if (length == 6) {
      insn |= (insn_bits_t)*(const int16_t*)translate_insn_addr_to_host(addr + 4) << 32;
      insn |= (insn_bits_t)*(const uint16_t*)translate_insn_addr_to_host(addr + 2) << 16;
    } else {
      static_assert(sizeof(insn_bits_t) == 8, "insn_bits_t must be uint64_t");
      insn |= (insn_bits_t)*(const int16_t*)translate_insn_addr_to_host(addr + 6) << 48;
      insn |= (insn_bits_t)*(const uint16_t*)translate_insn_addr_to_host(addr + 4) << 32;
      insn |= (insn_bits_t)*(const uint16_t*)translate_insn_addr_to_host(addr + 2) << 16;
    }

    insn_fetch_t fetch = {proc->decode_insn(insn), insn};
    entry->tag = addr;
    entry->data = fetch;

    reg_t paddr = tlb_entry.target_offset + addr;;
    if (tracer.interested_in_range(paddr, paddr + 1, FETCH)) {
      entry->tag = -1;
      tracer.trace(paddr, length, FETCH);
    }
    return entry;
  }

  inline icache_entry_t* access_icache(reg_t addr)
  {
    icache_entry_t* entry = &icache[icache_index(addr)];
    if (likely(entry->tag == addr))
      return entry;
    return refill_icache(addr, entry);
  }

  inline insn_fetch_t load_insn(reg_t addr)
  {
    icache_entry_t entry;
    return refill_icache(addr, &entry)->data;
  }

  void flush_tlb();
  void flush_icache();

  void flush_dcache() {
    dcache.flush();
  }
  void set_htif_addrs() {
    dcache.set_htif_addrs();
  }

  void register_memtracer(memtracer_t*);

private:
  sim_t* sim;
  processor_t* proc;
  memtracer_list_t tracer;
  uint16_t fetch_temp;

  // implement an instruction cache for simulator performance
  icache_entry_t icache[ICACHE_ENTRIES];

  // implement a TLB for simulator performance
  static const reg_t TLB_ENTRIES = 256;
  // If a TLB tag has TLB_CHECK_TRIGGERS set, then the MMU must check for a
  // trigger match before completing an access.
  static const reg_t TLB_CHECK_TRIGGERS = reg_t(1) << 63;
  tlb_entry_t tlb_data[TLB_ENTRIES];
  reg_t tlb_insn_tag[TLB_ENTRIES];
  reg_t tlb_load_tag[TLB_ENTRIES];
  reg_t tlb_store_tag[TLB_ENTRIES];

  // finish translation on a TLB miss and update the TLB
  tlb_entry_t refill_tlb(reg_t vaddr, reg_t paddr, char* host_addr, access_type type);
  const char* fill_from_mmio(reg_t vaddr, reg_t paddr);

  // perform a page table walk for a given VA; set referenced/dirty bits
  reg_t walk(reg_t addr, access_type type, reg_t prv);

  // handle uncommon cases: TLB misses, page faults, MMIO
  tlb_entry_t fetch_slow_path(reg_t addr);
  void load_slow_path(reg_t addr, reg_t len, uint8_t* bytes, bool exclusive, bool reserve);
  reg_t store_slow_path(reg_t addr, reg_t len, const uint8_t* bytes, bool conditional);
  reg_t translate(reg_t addr, access_type type);

  // ITLB lookup
  inline tlb_entry_t translate_insn_addr(reg_t addr) {
    reg_t vpn = addr >> PGSHIFT;
    if (likely(tlb_insn_tag[vpn % TLB_ENTRIES] == vpn))
      return tlb_data[vpn % TLB_ENTRIES];
    if (unlikely(tlb_insn_tag[vpn % TLB_ENTRIES] == (vpn | TLB_CHECK_TRIGGERS))) {
      uint16_t* ptr = (uint16_t*)(tlb_data[vpn % TLB_ENTRIES].host_offset + addr);
      int match = proc->trigger_match(OPERATION_EXECUTE, addr, *ptr);
      if (match >= 0)
        throw trigger_matched_t(match, OPERATION_EXECUTE, addr, *ptr);
      return tlb_data[vpn % TLB_ENTRIES];
    }
    return fetch_slow_path(addr);
  }

  inline const uint16_t* translate_insn_addr_to_host(reg_t addr) {
    return (uint16_t*)(translate_insn_addr(addr).host_offset + addr);
  }

  inline trigger_matched_t *trigger_exception(trigger_operation_t operation,
      reg_t address, reg_t data)
  {
    if (!proc) {
      return NULL;
    }
    int match = proc->trigger_match(operation, address, data);
    if (match == -1)
      return NULL;
    if (proc->state.mcontrol[match].timing == 0) {
      throw trigger_matched_t(match, operation, address, data);
    }
    return new trigger_matched_t(match, operation, address, data);
  }

  bool check_triggers_fetch;
  bool check_triggers_load;
  bool check_triggers_store;
  // The exception describing a matched trigger, or NULL.
  trigger_matched_t *matched_trigger;

  // write-through data cache
  siwt_cache_t dcache;

  friend class processor_t;
};

struct vm_info {
  int levels;
  int idxbits;
  int ptesize;
  reg_t ptbase;
};

inline vm_info decode_vm_info(int xlen, reg_t prv, reg_t sptbr)
{
  if (prv == PRV_M) {
    return {0, 0, 0, 0};
  } else if (prv <= PRV_S && xlen == 32) {
    switch (get_field(sptbr, SPTBR32_MODE)) {
      case SPTBR_MODE_OFF: return {0, 0, 0, 0};
      case SPTBR_MODE_SV32: return {2, 10, 4, (sptbr & SPTBR32_PPN) << PGSHIFT};
      default: abort();
    }
  } else if (prv <= PRV_S && xlen == 64) {
    switch (get_field(sptbr, SPTBR64_MODE)) {
      case SPTBR_MODE_OFF: return {0, 0, 0, 0};
      case SPTBR_MODE_SV39: return {3, 9, 8, (sptbr & SPTBR64_PPN) << PGSHIFT};
      case SPTBR_MODE_SV48: return {4, 9, 8, (sptbr & SPTBR64_PPN) << PGSHIFT};
      case SPTBR_MODE_SV57: return {5, 9, 8, (sptbr & SPTBR64_PPN) << PGSHIFT};
      case SPTBR_MODE_SV64: return {6, 9, 8, (sptbr & SPTBR64_PPN) << PGSHIFT};
      default: abort();
    }
  } else {
    abort();
  }
}

#endif
