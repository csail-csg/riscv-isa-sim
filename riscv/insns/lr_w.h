require_extension('A');
p->get_state()->load_reservation = RS1;
WRITE_RD(MMU.load_int32(RS1, true));
if (insn.aq()) MMU.flush_dcache();
