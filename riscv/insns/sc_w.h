require_extension('A');
reg_t v = MMU.store_uint32(RS1, RS2, true);
WRITE_RD(v);
if (insn.aq()) MMU.flush_dcache();
