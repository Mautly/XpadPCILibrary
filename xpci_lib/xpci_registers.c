//                 xpci_registers.c
//
// Unit to manage the registers identification as generic constants
// hiding the difference.
// This management is done as pure syntaxic manipulations which calls
// functions of this unit hidden from the end user in the xpci_lib.
//===================================================================
// PYD 14/02/2011
//==================================================================
#include "xpci_registers.h"
#include "xpci_interface.h"

#if defined(__cplusplus)
    extern "C" {
#endif

int xpci_reg_cmos_dsbl(int systemChip){
	if (systemChip==XPAD_31) return CMOS_DSBL_V31;    
	else if (systemChip==XPAD_32) return CMOS_DSBL_V32;  
	else return 0;
}
int xpci_reg_amp_tp(int systemChip){
	if (systemChip==XPAD_31) return AMP_TP_V31;    
	else if (systemChip==XPAD_32) return AMP_TP_V32;
	else return 0;
}

int xpci_reg_ithh(int systemChip){
	if (systemChip==XPAD_31) return ITHH_V31;    
	else if (systemChip==XPAD_32) return ITHH_V32;
	else return 0;
}    
int xpci_reg_vadj(int systemChip){
	if (systemChip==XPAD_31) return VADJ_V31;    
	else if (systemChip==XPAD_32) return VADJ_V32;
	else return 0;
}
int xpci_reg_vref(int systemChip){
	if (systemChip==XPAD_31) return VREF_V31;    
	else if (systemChip==XPAD_32) return VREF_V32; 
	else return 0;
}
int xpci_reg_imfp(int systemChip){
	if (systemChip==XPAD_31) return IMFP_V31;    
	else if (systemChip==XPAD_32) return IMFP_V32; 
	else return 0;
}
int xpci_reg_iota(int systemChip){
	if (systemChip==XPAD_31) return IOTA_V31;    
	else if (systemChip==XPAD_32) return IOTA_V32;
	else return 0;
}
int xpci_reg_ipre(int systemChip){
	if (systemChip==XPAD_31) return IPRE_V31;    
	else if (systemChip==XPAD_32) return IPRE_V32;
	else return 0;
}
int xpci_reg_ithl(int systemChip){
	if (systemChip==XPAD_31) return ITHL_V31;    
	else if (systemChip==XPAD_32) return ITHL_V32;
}
int xpci_reg_itune(int systemChip){
	if (systemChip==XPAD_31) return ITUNE_V31;    
	else if (systemChip==XPAD_32) return ITUNE_V32;  
	else return 0;
}
int xpci_reg_ibuffer(int systemChip){
	if (systemChip==XPAD_31) return IBUFFER_V31;    
	else if (systemChip==XPAD_32) return IBUFFER_V32;  
	else return 0;
}
#ifdef __cplusplus
}
#endif

#if defined(TEST)
#include <stdio.h>

int main(){
  int xpci_systemChip;
  printf("          V32       v31\n");
  xpci_systemChip = XPAD_32; printf("CMOS_DSBL=0x%x", CMOS_DSBL);
  xpci_systemChip = XPAD_31; printf("       0x%x\n",   CMOS_DSBL);

  xpci_systemChip = XPAD_32; printf("AMP_TP   =0x%x", AMP_TP);
  xpci_systemChip = XPAD_31; printf("      0x%x\n",   AMP_TP);

  xpci_systemChip = XPAD_32; printf("ITHH     =0x%x", ITHH);
  xpci_systemChip = XPAD_31; printf("      0x%x\n",   ITHH);

  xpci_systemChip = XPAD_32; printf("VADJ     =0x%x", VADJ);
  xpci_systemChip = XPAD_31; printf("      0x%x\n",   VADJ);

  xpci_systemChip = XPAD_32; printf("VREF     =0x%x", VREF);
  xpci_systemChip = XPAD_31; printf("      0x%x\n",   VREF);

  xpci_systemChip = XPAD_32; printf("ITHH     =0x%x", IMFP);
  xpci_systemChip = XPAD_31; printf("      0x%x\n",   IMFP);

  xpci_systemChip = XPAD_32; printf("IOTA     =0x%x", IOTA);
  xpci_systemChip = XPAD_31; printf("      0x%x\n",   IOTA);

  xpci_systemChip = XPAD_32; printf("IPRE     =0x%x", IPRE);
  xpci_systemChip = XPAD_31; printf("      0x%x\n",   IPRE);

  xpci_systemChip = XPAD_32; printf("ITHL     =0x%x", ITHL);
  xpci_systemChip = XPAD_31; printf("      0x%x\n",   ITHL);

  xpci_systemChip = XPAD_32; printf("ITUNE    =0x%x", ITUNE);
  xpci_systemChip = XPAD_31; printf("      0x%x\n",   ITUNE);

  xpci_systemChip = XPAD_32; printf("IBUFFER  =0x%x", IBUFFER);
  xpci_systemChip = XPAD_31; printf("      0x%x\n",   IBUFFER);

  return 0;
}

#endif
