`ifndef ECODES_VH
`define ECODES_VH

`define ECODEL  5:0
`define ECODEW  6
`define ESUBCODEL 8:0
`define ESUBCODEW 9

`define    ECODE_INT    6'h00   // interrupt (exception)
`define    ECODE_PIL    6'h01   // load page invalid exception
`define    ECODE_PIS    6'h02   // store page invalid exception
`define    ECODE_PIF    6'h03   // fetch page invalid exception
`define    ECODE_PME    6'h04   // page modified exception

`define    ECODE_PPI    6'h07   // page privilege invalid exception
`define    ECODE_ADE    6'h08   // address error exception
`define    ECODE_ALE    6'h09   // alignment error exception

`define    ECODE_SYS    6'h0b   // syscall exception
`define    ECODE_BRK    6'h0c   // breakpoint exception
`define    ECODE_INE    6'h0d   // instruction error exception
`define    ECODE_IPE    6'h0e   // instruction privilege exception
`define    ECODE_FPD    6'h0f   // floating point disabled exception

`define    ECODE_FPE    6'h12   // floating point exception

`define    ECODE_TLBR   6'h3f   // TLB refill exception

`define ESUBCODE_ADEF   9'h000  // address error on fetch
`define ESUBCODE_ADEM   9'h001  // address error on load/store

`endif // ECODES_VH