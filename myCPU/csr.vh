`define CSR_NUM_WIDTH 14

`define CSR_CRMD 14'h0
`define CSR_PRMD 14'h1
`define CSR_ECFG 14'h4
`define CSR_ESTAT 14'h5
`define CSR_ERA 14'h6
`define CSR_BADV 14'h7
`define CSR_EENTRY 14'hc
`define CSR_SAVE0 14'h30
`define CSR_SAVE1 14'h31
`define CSR_SAVE2 14'h32
`define CSR_SAVE3 14'h33
`define CSR_TID 14'h40
`define CSR_TCFG 14'h41
`define CSR_TVAL 14'h42
`define CSR_TICLR 14'h44

// BITS in CRMD
`define CSR_CRMD_PLV 1:0
`define CSR_CRMD_IE 2:2
`define CSR_CRMD_DA 3:3
`define CSR_CRMD_PG 4:4
`define CSR_CRMD_DATF 6:5
`define CSR_CRMD_PGF 8:7

// BITS in PRMD
`define CSR_PRMD_PPLV 1:0
`define CSR_PRMD_PIE 2:2

// BITS in ECFG
`define CSR_ECFG_LIE 12:0

// BITS in ESTAT
`define CSR_ESTAT_IS 12:0
`define CSR_ESTAT_IS_1_0 1:0
`define CSR_ESTAT_IS_HARD 9:2

// BITS in ERA
`define CSR_ERA_PC 31:0

// BITS in EENTRY
`define CSR_EENTRY_VA 31:6

// BITS in SAVE
`define CSR_SAVE0_DATA 31:0
`define CSR_SAVE1_DATA 31:0
`define CSR_SAVE2_DATA 31:0
`define CSR_SAVE3_DATA 31:0

// BITS in TCFG
`define CSR_TCFG_EN 0:0
`define CSR_TCFG_PERIODIC 1:1
`define CSR_TCFG_INITVAL 31:2

// BITS in TICLR
`define CSR_TICLR_CLR 0:0
