require_extension('A');
require_rv64;
reg_t v = MMU.store_uint64(RS1, RS2, true);
WRITE_RD(v);
if (insn.aq()) MMU.flush_dcache();
