// See LICENSE for license details.

#include "sim.h"
#include "mmu.h"
#include "remote_bitbang.h"
#include <map>
#include <iostream>
#include <sstream>
#include <climits>
#include <cstdlib>
#include <cassert>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>

volatile bool ctrlc_pressed = false;
static void handle_signal(int sig)
{
  if (ctrlc_pressed)
    exit(-1);
  ctrlc_pressed = true;
  signal(sig, &handle_signal);
}

sim_t::sim_t(const char* isa, size_t nprocs, bool halted, reg_t start_pc,
             std::vector<std::pair<reg_t, mem_t*>> _mems,
             const char *rom_bin,
             const char *verify_log,
             addr_t verify_terminate_pc,
             const std::vector<std::string>& args)
  : htif_t(args), debug_module(this), mems(_mems), procs(std::max(nprocs, size_t(1))),
    start_pc(start_pc), rom_bin_file(rom_bin),
    current_step(0), current_proc(0), debug(false), remote_bitbang(NULL),
    // tandem verify
    verify_log_fp(NULL),
    verify_stop_pc(verify_terminate_pc),
    verify_stopped(false),
    verify_icount(0)
{
  signal(SIGINT, &handle_signal);

  for (auto& x : mems)
    bus.add_device(x.first, x.second);

  debug_module.add_device(&bus);

  debug_mmu = new mmu_t(this, NULL);

  for (size_t i = 0; i < procs.size(); i++) {
    procs[i] = new processor_t(isa, this, i, halted);
  }

  clint.reset(new clint_t(procs));
  bus.add_device(CLINT_BASE, clint.get());

  if (verify_log) {
      verify_log_fp = fopen(verify_log, "w");
      if (!verify_log_fp) {
          fprintf(stderr, "Fail to open %s\n", verify_log);
          exit(-1);
      }
  }
}

sim_t::~sim_t()
{
  for (size_t i = 0; i < procs.size(); i++)
    delete procs[i];
  delete debug_mmu;
}

void sim_thread_main(void* arg)
{
  ((sim_t*)arg)->main();
}

void sim_t::main()
{
  if (!debug && log)
    set_procs_debug(true);

  while (!done())
  {
    if (debug || ctrlc_pressed)
      interactive();
    else
      step(INTERLEAVE);
    if (remote_bitbang) {
      remote_bitbang->tick();
    }
  }
}

int sim_t::run()
{
  host = context_t::current();
  target.init(sim_thread_main, this);
  return htif_t::run();
}

void sim_t::step(size_t n)
{
  for (size_t i = 0, steps = 0; i < n; i += steps)
  {
    steps = std::min(n - i, INTERLEAVE - current_step);
    bool unused;
    procs[current_proc]->step(steps, unused);

    current_step += steps;
    if (current_step == INTERLEAVE)
    {
      current_step = 0;
      procs[current_proc]->yield_load_reservation();
      if (++current_proc == procs.size()) {
        current_proc = 0;
        clint->increment(INTERLEAVE / INSNS_PER_RTC_TICK);
      }

      host->switch_to();
    }
  }
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
  if (addr + len < addr)
    return false;
  return bus.load(addr, len, bytes);
}

bool sim_t::mmio_store(reg_t addr, size_t len, const uint8_t* bytes)
{
  if (addr + len < addr)
    return false;
  return bus.store(addr, len, bytes);
}

static std::string dts_compile(const std::string& dts)
{
  // Convert the DTS to DTB
  int dts_pipe[2];
  pid_t dts_pid;

  if (pipe(dts_pipe) != 0 || (dts_pid = fork()) < 0) {
    std::cerr << "Failed to fork dts child: " << strerror(errno) << std::endl;
    exit(1);
  }

  // Child process to output dts
  if (dts_pid == 0) {
    close(dts_pipe[0]);
    int step, len = dts.length();
    const char *buf = dts.c_str();
    for (int done = 0; done < len; done += step) {
      step = write(dts_pipe[1], buf+done, len-done);
      if (step == -1) {
        std::cerr << "Failed to write dts: " << strerror(errno) << std::endl;
        exit(1);
      }
    }
    close(dts_pipe[1]);
    exit(0);
  }

  pid_t dtb_pid;
  int dtb_pipe[2];
  if (pipe(dtb_pipe) != 0 || (dtb_pid = fork()) < 0) {
    std::cerr << "Failed to fork dtb child: " << strerror(errno) << std::endl;
    exit(1);
  }

  // Child process to output dtb
  if (dtb_pid == 0) {
    dup2(dts_pipe[0], 0);
    dup2(dtb_pipe[1], 1);
    close(dts_pipe[0]);
    close(dts_pipe[1]);
    close(dtb_pipe[0]);
    close(dtb_pipe[1]);
    execl(DTC, DTC, "-O", "dtb", 0);
    std::cerr << "Failed to run " DTC ": " << strerror(errno) << std::endl;
    exit(1);
  }

  close(dts_pipe[1]);
  close(dts_pipe[0]);
  close(dtb_pipe[1]);

  // Read-out dtb
  std::stringstream dtb;

  int got;
  char buf[4096];
  while ((got = read(dtb_pipe[0], buf, sizeof(buf))) > 0) {
    dtb.write(buf, got);
  }
  if (got == -1) {
    std::cerr << "Failed to read dtb: " << strerror(errno) << std::endl;
    exit(1);
  }
  close(dtb_pipe[0]);

  // Reap children
  int status;
  waitpid(dts_pid, &status, 0);
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    std::cerr << "Child dts process failed" << std::endl;
    exit(1);
  }
  waitpid(dtb_pid, &status, 0);
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    std::cerr << "Child dtb process failed" << std::endl;
    exit(1);
  }

  return dtb.str();
}

void sim_t::make_dtb(std::vector<char> &rom)
{
  std::stringstream s;
  s << std::dec <<
         "/dts-v1/;\n"
         "\n"
         "/ {\n"
         "  #address-cells = <2>;\n"
         "  #size-cells = <2>;\n"
         "  compatible = \"ucbbar,spike-bare-dev\";\n"
         "  model = \"ucbbar,spike-bare\";\n"
         "  cpus {\n"
         "    #address-cells = <1>;\n"
         "    #size-cells = <0>;\n"
         "    timebase-frequency = <" << (CPU_HZ/INSNS_PER_RTC_TICK) << ">;\n";
  for (size_t i = 0; i < procs.size(); i++) {
    s << "    CPU" << i << ": cpu@" << i << " {\n"
         "      device_type = \"cpu\";\n"
         "      reg = <" << i << ">;\n"
         "      status = \"okay\";\n"
         "      compatible = \"riscv\";\n"
         "      riscv,isa = \"" << procs[i]->isa_string << "\";\n"
         "      mmu-type = \"riscv," << (procs[i]->max_xlen <= 32 ? "sv32" : "sv39") << "\";\n"
         "      clock-frequency = <" << CPU_HZ << ">;\n"
         "      CPU" << i << "_intc: interrupt-controller {\n"
         "        #interrupt-cells = <1>;\n"
         "        interrupt-controller;\n"
         "        compatible = \"riscv,cpu-intc\";\n"
         "      };\n"
         "    };\n";
  }
  s <<   "  };\n";
  for (auto& m : mems) {
    s << std::hex <<
         "  memory@" << m.first << " {\n"
         "    device_type = \"memory\";\n"
         "    reg = <0x" << (m.first >> 32) << " 0x" << (m.first & (uint32_t)-1) <<
                   " 0x" << (m.second->size() >> 32) << " 0x" << (m.second->size() & (uint32_t)-1) << ">;\n"
         "  };\n";
  }
  s <<   "  soc {\n"
         "    #address-cells = <2>;\n"
         "    #size-cells = <2>;\n"
         "    compatible = \"ucbbar,spike-bare-soc\", \"simple-bus\";\n"
         "    ranges;\n"
         "    clint@" << CLINT_BASE << " {\n"
         "      compatible = \"riscv,clint0\";\n"
         "      interrupts-extended = <" << std::dec;
  for (size_t i = 0; i < procs.size(); i++)
    s << "&CPU" << i << "_intc 3 &CPU" << i << "_intc 7 ";
  reg_t clintbs = CLINT_BASE;
  reg_t clintsz = CLINT_SIZE;
  s << std::hex << ">;\n"
         "      reg = <0x" << (clintbs >> 32) << " 0x" << (clintbs & (uint32_t)-1) <<
                     " 0x" << (clintsz >> 32) << " 0x" << (clintsz & (uint32_t)-1) << ">;\n"
         "    };\n"
         "  };\n"
         "};\n";

  dts = s.str();
  std::string dtb = dts_compile(dts);

  rom.insert(rom.end(), dtb.begin(), dtb.end());
  const int align = 0x1000;
  rom.resize((rom.size() + align - 1) / align * align);

  boot_rom.reset(new rom_device_t(rom));
  bus.add_device(DEFAULT_RSTVEC, boot_rom.get());

  fprintf(stderr, "info: dts:\n%s", dts.c_str());
  fprintf(stderr, "info: boot rom size = %d\n", int(boot_rom->contents().size()));
}

char* sim_t::addr_to_mem(reg_t addr) {
  auto desc = bus.find_device(addr);
  if (auto mem = dynamic_cast<mem_t*>(desc.second))
    if (addr - desc.first < mem->size())
      return mem->contents() + (addr - desc.first);
  return NULL;
}

// htif

void sim_t::reset()
{
  std::vector<char> rom;

  if (rom_bin_file) {
        // load from file
        FILE *fp = fopen(rom_bin_file, "rb");
        if(!fp) {
            fprintf(stderr, "ERROR: Fail to open %s\n", rom_bin_file);
            exit(-1);
        }
        const size_t buffer_size = 256;
        char buffer[buffer_size];
        while(true) {
            size_t cnt = fread(buffer, 1, buffer_size, fp);
            rom.insert(rom.end(), buffer, buffer + cnt);
            if(cnt % 8) {
                fprintf(stderr, "ERROR: Read %d bytes from boot rom\n", int(cnt));
                exit(-1);
            }
            if(cnt < buffer_size) {
                break;
            }
        }
  }
  else {
    const int reset_vec_size = 8;

    start_pc = start_pc == reg_t(-1) ? get_entry_point() : start_pc;

    uint32_t reset_vec[reset_vec_size] = {
      0x297,                                      // auipc  t0,0x0
      0x28593 + (reset_vec_size * 4 << 20),       // addi   a1, t0, &dtb
      0xf1402573,                                 // csrr   a0, mhartid
      get_core(0)->xlen == 32 ?
        0x0182a283u :                             // lw     t0,24(t0)
        0x0182b283u,                              // ld     t0,24(t0)
      0x28067,                                    // jr     t0
      0,
      (uint32_t) (start_pc & 0xffffffff),
      (uint32_t) (start_pc >> 32)
    };

    rom.insert(rom.end(), (char*)reset_vec, (char*)reset_vec + sizeof(reset_vec));
  }

  make_dtb(rom);
}

void sim_t::idle()
{
  target.switch_to();
}

void sim_t::read_chunk(addr_t taddr, size_t len, void* dst)
{
  assert(len == 8);
  auto data = debug_mmu->load_uint64(taddr);
  memcpy(dst, &data, sizeof data);
}

void sim_t::write_chunk(addr_t taddr, size_t len, const void* src)
{
  assert(len == 8);
  uint64_t data;
  memcpy(&data, src, sizeof data);
  debug_mmu->store_uint64(taddr, data);
}

bool sim_t::verify(reg_t icount, reg_t pc, reg_t next_pc, uint32_t inst,
                   bool src1_valid, int src1_reg, reg_t src1_data,
                   bool src2_valid, int src2_reg, reg_t src2_data,
                   bool dst_valid, int dst_reg, reg_t dst_data,
                   bool is_ld, bool is_st, addr_t paddr, uint32_t align_be,
                   reg_t align_st_data, reg_t align_ld_data) {
    // assume only 1 core
    processor_t *cur_proc = procs[0];

    if (unlikely(verify_stopped)) {
        return true;
    }

    // log verification
    if (likely(verify_log_fp != NULL)) {
        fprintf(verify_log_fp,
                "icount %llu, pc %016llx, next pc %016llx, inst %08x, "
                "src1_valid %d, src1_reg %d, src1_data %016llx, "
                "src2_valid %d, src2_reg %d, src2_data %016llx, "
                "dst_valid %d, dst_reg %d, dst_data %016llx, "
                "is_ld %d, is_st %d, paddr %016llx, align_be %02x, "
                "align_st_data %016llx, align_ld_data %016llx\n",
                (long long unsigned)icount,
                (long long unsigned)pc,
                (long long unsigned)next_pc,
                (unsigned)inst,
                (int)src1_valid, src1_reg, (long long unsigned)src1_data,
                (int)src2_valid, src2_reg, (long long unsigned)src2_data,
                (int)dst_valid, dst_reg, (long long unsigned)dst_data,
                (int)is_ld, (int)is_st,
                (long long unsigned)paddr, (unsigned)align_be,
                (long long unsigned)align_st_data,
                (long long unsigned)align_ld_data);
    }

    // check in M mode
    if (unlikely(cur_proc->state.prv != PRV_M)) {
        fprintf(stderr, "[TANDEM VERIFY] ERROR: in Non-M mode %lld\n",
                (long long unsigned)(cur_proc->state.prv));
        stop_verify();
        return false;
    }
    // check icount match
    if (unlikely(icount != verify_icount)) {
        fprintf(stderr, "[TANDEM VERIFY] ERROR: icount should be %llu\n",
                (long long unsigned)verify_icount);
        stop_verify();
        return false;
    }
    // check pc
    if (unlikely(pc != cur_proc->state.pc)) {
        fprintf(stderr, "[TANDEM VERIFY] ERROR: pc should be %016llx\n",
                (long long unsigned)(cur_proc->state.pc));
        stop_verify();
        return false;
    }
    // check stop
    if (unlikely(pc == verify_stop_pc)) {
        fprintf(stderr, "[TANDEM VERIFY] Stop at pc %016llx\n",
                (long long unsigned)pc);
        stop_verify();
        return true;
    }
    // check inst
    uint32_t ref_inst = debug_mmu->load_uint32(pc);
    if (unlikely(inst != ref_inst)) {
        fprintf(stderr, "[TANDEM VERIFY] ERROR: inst should be %08x\n",
                (unsigned)ref_inst);
        stop_verify();
        return false;
    }
    // check src values
    if (src1_valid) {
        assert(src1_reg >= 0 && src1_reg < NXPR);
        reg_t val = cur_proc->state.XPR[src1_reg];
        if (unlikely(src1_data != val)) {
            fprintf(stderr, "[TANDEM VERIFY] ERROR: src1 data should be %016llx\n",
                    (long long unsigned)val);
            stop_verify();
            return false;
        }
    }
    if (src2_valid) {
        assert(src2_reg >= 0 && src2_reg < NXPR);
        reg_t val = cur_proc->state.XPR[src2_reg];
        if (unlikely(src2_data != val)) {
            fprintf(stderr, "[TANDEM VERIFY] ERROR: src2 data should be %016llx\n",
                    (long long unsigned)val);
            stop_verify();
            return false;
        }
    }
    // get memory value
    reg_t mem_val = 0;
    addr_t align_paddr = paddr & ~reg_t(0x07);
    if (is_ld || is_st) {
        assert(!(is_ld && is_st));
        mem_val = debug_mmu->load_uint64(align_paddr);
        if (is_ld) {
            uint8_t *p_ld = (uint8_t*)(&align_ld_data);
            uint8_t *p_mem = (uint8_t*)(&mem_val);
            for (int i = 0; i < sizeof(reg_t); i++) {
                if ((align_be >> i) & 0x01) {
                    if (unlikely(p_ld[i] != p_mem[i])) {
                        fprintf(stderr, "[TANDEM VERIFY] ERROR: "
                                "Ld data should be %016llx\n",
                                (long long unsigned)mem_val);
                        stop_verify();
                        return false;
                    }
                }
            }
        }
    }

    // step 1 inst
    bool trapped = false;
    cur_proc->step(1, trapped);
    verify_icount++;

    // check no trap
    if (unlikely(trapped)) {
        fprintf(stderr, "[TANDEM VERIFY] ERROR: unexpected trap\n");
        stop_verify();
        return false;
    }
    // check next pc
    if (unlikely(next_pc != cur_proc->state.pc)) {
        fprintf(stderr, "[TANDEM VERIFY] ERROR: next pc should be %016llx\n",
                (long long unsigned)(cur_proc->state.pc));
        stop_verify();
        return false;
    }
    // check dst reg value
    if (dst_valid) {
        assert(dst_reg >= 0 && dst_reg < NXPR);
        reg_t val = cur_proc->state.XPR[dst_reg];
        if (unlikely(dst_data != val)) {
            fprintf(stderr, "[TANDEM VERIFY] ERROR: dst data should be %016llx\n",
                    (long long unsigned)val);
            stop_verify();
            return false;
        }
    }
    // check store value
    if (is_st) {
        reg_t ref_mem_val = debug_mmu->load_uint64(align_paddr);
        // compute new mem val using store data
        reg_t new_mem_val = mem_val;
        uint8_t *p_mem = (uint8_t*)(&new_mem_val);
        uint8_t *p_st = (uint8_t*)(&align_st_data);
        for (int i = 0; i < sizeof(reg_t); i++) {
            if ((align_be >> i) & 0x01) {
                p_mem[i] = p_st[i];
            }
        }
        if (unlikely(new_mem_val != ref_mem_val)) {
            fprintf(stderr, "[TANDEM VERIFY] ERROR: St should change "
                    "memory [%016llx] = %016llx to %016llx, but to %016llx\n",
                    (long long unsigned)align_paddr,
                    (long long unsigned)mem_val,
                    (long long unsigned)ref_mem_val,
                    (long long unsigned)new_mem_val);
            stop_verify();
            return false;
        }
    }

    // everything looks good
    return true;
}

