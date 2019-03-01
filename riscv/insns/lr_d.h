require_extension('A');
require_rv64;
WRITE_RD(MMU.load_int64(RS1, true, true));
if (insn.aq()) MMU.flush_dcache();
