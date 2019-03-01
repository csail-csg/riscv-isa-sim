require_extension('A');
WRITE_RD(MMU.load_int32(RS1, true, true));
if (insn.aq()) MMU.flush_dcache();
