//                 xpci_registers.h
//
// Unit to manage the registers identification as generic constants
// hiding the difference.
// This management is done as pure syntaxic manipulations as defined 
// in this unit hidden from the end user.
//===================================================================
// PYD 14/02/2011
//==================================================================
#ifndef XPCI_REG
#define XPCI_REG

#if defined(__cplusplus)
    extern "C" {
#endif

/* CHIP REGISTERS CODE XPAD31 */
#define CMOS_DSBL_V31  0x01
#define AMP_TP_V31     0x32
#define ITHH_V31       0x33
#define VADJ_V31       0x35
#define VREF_V31       0x36
#define IMFP_V31       0x37
#define IOTA_V31       0x38
#define IPRE_V31       0x3B
#define ITHL_V31       0x3C
#define ITUNE_V31      0x41
#define IBUFFER_V31    0x42 

/* CHIP REGISTERS CODE XPAD32 */
#define CMOS_DSBL_V32  0x01
#define AMP_TP_V32     0x1F
#define ITHH_V32       0x33
#define VADJ_V32       0x35
#define VREF_V32       0x36
#define IMFP_V32       0x3b
#define IOTA_V32       0x3c
#define IPRE_V32       0x3d
#define ITHL_V32       0x3e
#define ITUNE_V32      0x3f
#define IBUFFER_V32    0x40



      int xpci_reg_cmos_dsbl(int systemChip);
      int xpci_reg_amp_tp(int systemChip);
      int xpci_reg_ithh(int systemChip);
      int xpci_reg_vadj(int systemChip);
      int xpci_reg_vref(int systemChip);
      int xpci_reg_imfp(int systemChip);
      int xpci_reg_iota(int systemChip);
      int xpci_reg_ipre(int systemChip);
      int xpci_reg_ithl(int systemChip);
      int xpci_reg_itune(int systemChip);
      int xpci_reg_ibuffer(int systemChip);

#ifdef __cplusplus
}
#endif
#endif
