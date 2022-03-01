#ifndef XPCI_EXP
#define XPCI_EXP

#include <stdint.h>
/***********************************************************************************
//             Structures definition
***********************************************************************************/
typedef struct StatusRegTable StatusRegTable;

#if defined(__cplusplus)
    extern "C" {
#endif

      /*============================================================================================*/
      /*                            EXPERT DEDICATED FUNCTIONS                                         */
      /*============================================================================================*/

void  xpci_debugMsg(int flag);

/* XPCI hardware system management level functions for expert tests */
int   xpci_setHubLoopMode();
int   xpci_resetHubLoopMode();

/* low level board driverfunctions  and associated PC memory management functions */
/* PXIe users are not supposed to use them                                        */
int   xpci_writeCommon(uint16_t *data, int size);
int   xpci_writeCommon_S1400(uint16_t *data, int size,unsigned modMask);
int   xpci_write(int channel, uint16_t *data, int size);
int   xpci_writeTestPCI(int channel, uint16_t *data, int size);
int   xpci_read(int channel, uint16_t *data, int size, int timeout);
int   xpci_modReadConfig(unsigned modMask);
int   xpci_resetBoard(int det);
int   xpci_resetChannels(int channel);
void  xpci_getStatusRegsTable(StatusRegTable **regs);
void  xpci_getFifoFillingStatus(int *level0, int *level1);
int   xpci_getInterruptStatus();
void  xpci_dumpStatusRegsTable();
int   xpci_setPCILoopMode();
int   xpci_resetPCILoopMode();
int   xpci_getDmaStatus(int channel, int print);

int   waitCommandReply(unsigned modMask, char *userFunc, int timeout);


/* internal function used for combining expose+read managed directly by the detector nloop images */
int   xpci_startImgSequence(unsigned nloop);
int   xpci_readImgBuff(void *data, int timeout);
void  xpci_getImageClose();
int   xpci_modImageGet2B(unsigned modMask, unsigned gateMode, unsigned gateLength, unsigned timeUnit);
int   xpci_modImageGet2B_XPAD32(unsigned modMask, unsigned gateMode, unsigned gateLength, unsigned timeUnit, unsigned nloop);
int   xpci_modImageGet4B(unsigned modMask, unsigned gateMode, unsigned gateLength, unsigned timeUnit);

/* CPPM implementation */
int   xpci_getImgSeq_CPPM(enum IMG_TYPE type, int moduleMask, int nbChips,
		         int gateMode, int gateLength, int timeUnit,
		         int nbImg, void **pBuff, int firstTimeout);
int   xpci_readOneImage_CPPM(enum IMG_TYPE type, int moduleMask, int nbChips, void *data);
int   xpci_getModConfig_CPPM(unsigned moduleMask, unsigned nbChips, uint16_t *data);

/* imXPAD implementation */
int   xpci_getImgSeq_imxpad(enum IMG_TYPE type, int moduleMask, int nbChips,
		           int nbImg, void **pBuff);
int   xpci_readOneImage_imxpad(enum IMG_TYPE type, int moduleMask, int nbChips, void *data);
int   xpci_getModConfig_imxpad(unsigned moduleMask, unsigned nbChips, uint16_t *data);
int   xpci_imxpadModRebootNIOS();
int   waitCommandReplyExtended(unsigned modMask, char *userFunc, int timeout, unsigned *detRet);
int   xpix_imxpadWriteSubchnlReg(unsigned modMask, unsigned msgType, unsigned trloops);

/* low level debugging functions for expert */
int xpci_getItCnt();
int xpci_getTotalItCnt();

/*****************************************************************************
functions to emulate low level access to detector à la usbwrap.h
*******************************************************************************/
int   xpci_openPci(int det, int sysType);
int   xpci_readData(int det, int module, int *data, long unsigned int length );
int   xpci_writeData(int det, int module, int *data, long unsigned int length );
int   xpci_closePci(int det);


/***********************************************
                 deep debug
***********************************************/
int   xpci_modSingleAskReady(int module); // added by Arek

int  xpci_arekReadImage(int moduleMask, int nbChips, uint16_t *data, int loop, int fileFlag);
void xpci_itStatus();
void printUserBuffer(unsigned *userAddr, int nbLine);
void initIt();
void spinWaitOnIT();
int  getItCnt();
int  getItCnt2();

#ifdef __cplusplus
}
#endif

#endif

