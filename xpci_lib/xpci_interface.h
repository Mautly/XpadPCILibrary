#ifndef XPCI
#define XPCI

// necessary for use type 
#include <stdint.h>
#include "xpci_registers.h"

#include <sys/mman.h>       //for shm_open
#include <fcntl.h>          // For O_* constants
#include <errno.h>
#include <limits.h>

#if defined(__cplusplus)
extern "C" {
#endif

/*     TYPE OF SYTEM     */

#define HUB                         0
#define BACKPLANE                   200  
#define IMXPAD_S340                 201
#define IMXPAD_S420                 202


#define IMXPAD_S70                  101 
#define IMXPAD_S140                 102
#define IMXPAD_S540                 103
#define IMXPAD_S700                 104
#define IMXPAD_S1400                105

/*     TYPE OF CHIPS     */
#define XPAD_31                     0
#define XPAD_32                     1  

/* IMAGE POST PROCESSING */
#define IMG_POSTPROC_GEOM           0x00000001
#define IMG_POSTPROC_DEAD           0x00000002

/*****     CHIP REGISTERS MANAGEMENT CODE      ****/
/*** see xpci_registers.h and xpci_registers.c  ***/
/**************************************************/
extern int xpci_systemChip; // necessary for compilation
#define CMOS_DSBL  xpci_reg_cmos_dsbl(xpci_systemChip)
#define AMP_TP     xpci_reg_amp_tp(xpci_systemChip)
#define ITHH       xpci_reg_ithh(xpci_systemChip)
#define VADJ       xpci_reg_vadj(xpci_systemChip)
#define VREF       xpci_reg_vref(xpci_systemChip)
#define IMFP       xpci_reg_imfp(xpci_systemChip)
#define IOTA       xpci_reg_iota(xpci_systemChip)
#define IPRE       xpci_reg_ipre(xpci_systemChip)
#define ITHL       xpci_reg_ithl(xpci_systemChip)
#define ITUNE      xpci_reg_itune(xpci_systemChip)
#define IBUFFER    xpci_reg_ibuffer(xpci_systemChip)

#define IMG_SSD	300

/* GATE MODE */
#define INTERN_GATE 0x0
#define EXTERN_GATE 0x1
#define TRIGGER_IN     0xA

/* INTERNAL GATE UNIT */
#define MICROSEC_GATE 0x1
#define MILLISEC_GATE 0x2
#define SECONDS_GATE  0x3

enum    DATA_TYPE {IMG, CONFIG};
enum    IMG_TYPE  {B2,B4};

/*============================================================================================*/
/*                            USER ORIENTED FUNCTIONS                                         */
/*============================================================================================*/

int   xpci_init(int det, int sysType);
void  xpci_close(int det);

/* Test function for hardware availability */
int   xpci_isPCIeOK();
int   xpci_isHubOK();

/* XPCI hardware system management level functions to reset the system components by software */
// HUB version
int   xpci_hubReset();
int   xpci_hubModRebootNIOS(unsigned mask);
int   xpci_hubModReconfFPGA(unsigned mask);
// Backplane version
int   xpci_resetFifoPciGx();
int   xpci_modRebootNIOS(unsigned mask);


// ALBA detector commands
static int xpci_regModuleName();
static int xpci_regRstFifo();


/* XPIX detector commands */
int   xpci_modAskReady(unsigned *inMask);
int   xpci_modGlobalAskReady(unsigned modMask);
int   xpci_modLoadAutoTest(unsigned targetMask, unsigned value, unsigned mode);
int   xpci_modLoadConfigG(unsigned modMask, unsigned chipMask, 
                          unsigned reg, unsigned regVal);
int   xpci_modLoadAllConfigG(unsigned modMask, unsigned chipMask,
                             unsigned cmosVal,
                             unsigned amptpVal,
                             unsigned ithhVal,
                             unsigned vadjVal,
                             unsigned vrefVal,
                             unsigned imfpVal,
                             unsigned iotaVal,
                             unsigned ipreVal,
                             unsigned ithlVal,
                             unsigned ituneVal,
                             unsigned iBuffer
                             );
int   xpci_modLoadFlatConfig(unsigned modMask, unsigned chipMask, unsigned value);
int   xpci_modImage2BReq(unsigned modMask);
int   xpci_modImage4BReq(unsigned modMask);
int   xpci_getModConfig(unsigned moduleMask, unsigned nbChips, uint16_t *data);
int   xpci_modSaveConfigL(unsigned modMask, unsigned calibId, unsigned chipId, unsigned curRow, unsigned *value);
int   xpci_modSaveConfigG(unsigned modMask, unsigned calibId, unsigned reg, unsigned *values);
int   xpci_modDetLoadConfig(unsigned modMask, unsigned calibId);
int   xpci_modExpose(unsigned modMask, unsigned gateMode, unsigned gateLength, unsigned timeUnit, int timeout);

/* calibration pulse functions */
int   xpci_modPulseFlat(unsigned modMask, unsigned chipMask, unsigned valuedacl,unsigned amplPulse,unsigned NbHit);
int   xpci_modPulseConfig(unsigned modMask, unsigned chipMask, unsigned calibID,unsigned amplPulse,unsigned NbHit);

/* message and image transmission timeout management */ 
void  xpci_setHardTimeout(int value);
int   xpci_getHardTimeout();

/* pure image reading functions without exposition (digital test) */
int   xpci_readOneImage(enum IMG_TYPE type, int moduleMask, int nbChips, void *data);

/* function combining expose+read managed directly by the detector nloop images */
/* top function */
int   xpci_getImgSeq(enum IMG_TYPE type, int moduleMask, int nbChips, int nbImg, void **pBuff,
                     int gateMode_CPPM, int gateLength_CPPM, int timeUnit_CPPM, int firstTimeout_CPPM);
/*Streaming*/
int   xpci_getImgSeq_SSD_imxpad(enum IMG_TYPE type, int modMask, int nImg, int burstNumber);

/* function combining expose+read by sofware to get an image */
int   xpci_getOneImage(enum IMG_TYPE type, int moduleMask, int nbChips, void *data,
                       int gateMode, int gateLength, int timeUnit, int timeout);

/* async functions */
int   xpci_getGotImages();
int   xpci_readOneImageAs(enum IMG_TYPE type,
                          int moduleMask, int nbChips, void *data,
                          int (*cbFunc)(int myint, void *dum), int timeout,
                          void * userPara);
int   xpci_getOneImageAs(enum IMG_TYPE type, 
                         int moduleMask, int nbChips, void *data,
                         int (*cbFunc)(int myint, void *dum), int timeout,
                         int gateMode, int gateLength, int timeUnit,
                         void *userPara);

int   xpci_asyncReadStatus();
int   xpci_getImgSeqAs(enum IMG_TYPE type, int moduleMask, int nbChips,
                       int (*cbFunc)(int myint, void *dum), int timeout,
                       int gateMode, int gateLength, int timeUnit,
                       int nloop, void **pBuff, int firstTimeout,
                       void *userPara, int nImg);
int   xpci_getImgSeqAsync(enum IMG_TYPE type, int moduleMask,int nImg, int burstNumber);
int   xpci_getAsyncImageFromSharedMemory(enum IMG_TYPE type, int modMask, int nChips, int nImg, void *pBuff, int imageToGet, void *pImgCorr, int geomCorr);
int   xpci_getAsyncImageFromDisk(enum IMG_TYPE type, int modMask, void *pImg, int imageToGet, int burstNumber);
int   xpci_getNumberLastAcquiredAsyncImage();
void  xpci_clearNumberLastAcquiredAsyncImage();
void  xpci_cleanSharedMemory();
void  xpci_cleanSSDImages(unsigned int burstNumber, unsigned int imagesNumber);
int   xpci_modReadTempSensor(unsigned modMask, float *detRet);

/* imxpad functions */
int   xpci_modExposureParam( unsigned modMask,unsigned Texp,unsigned Twait,unsigned Tinit,
                             unsigned Tshutter,unsigned Tovf,unsigned mode, unsigned n,unsigned p,
                             unsigned nbImages,unsigned BusyOutSel,unsigned formatIMG,unsigned postProc,
                             unsigned GP1,unsigned AcqMode,unsigned StakingOrBunchMode,unsigned GP4);
                             
int xpci_modExposureParam_internal( unsigned modMask,unsigned Texp,unsigned Twait,unsigned Tinit,
									unsigned Tshutter,unsigned Tovf,unsigned mode,unsigned n,unsigned p,
									unsigned nbImages,unsigned BusyOutSel,unsigned formatIMG,unsigned postProc,
									unsigned GP1,unsigned AcqMode,unsigned StakingOrBunchMode,unsigned GP4);
									                             
int   xpci_modSendExpWaitTimes(unsigned modMask, unsigned startAddr, unsigned endAddr, unsigned *waitTimes);
int   xpci_sendIPIParam(unsigned modMask, unsigned IPI_ena, unsigned IPI_nr);
int   xpci_sendIPI(unsigned modMask, unsigned nloop);
int   xpci_modReadConfigG(unsigned modMask, unsigned chipMask, unsigned reg, unsigned *detRet);
int   xpci_pulserImxpad(unsigned modMask, unsigned Pnum, unsigned Pmask);
int   xpci_digitalTest(int modMask, int nbChips, uint16_t *pBuff, unsigned value, unsigned mode);
int   xpci_modAbortExposure();
void  xpci_AbortCleanDetector(unsigned modMask);

/* commodity for image reading support and parametrization */
void  xpci_getImgDataParameters(enum IMG_TYPE type, int nbChips, int *lineSize, int *img_tranferSize, int *img_sizeImage);
int   xpci_getModNb(unsigned modMask);
int   xpci_getFirstMod(unsigned modMask);
int   xpci_getLastMod(unsigned modMask);
void  xpadModuleType( int *modules_nb , int *start_module);
int   xpci_getModNbOnChnl(unsigned modMask, int channel);


/* state and error management */
int   xpci_getSystemErrorCode();
char *xpci_getSystemStatus();

void* xpci_writeRawDataToFile(unsigned int *par); // fred
void xpci_setAbortProcess();
void xpci_clearAbortProcess();
int  xpci_getAbortProcess();
unsigned int  xpci_getImageFormat(void);
unsigned int get_flagStartExpose (void);


void xpci_setItCount();
void xpci_clearItCount();
int xpci_getItCount();
unsigned int getFirmwareId(void);

int xpci_modMemDiag(unsigned modMask,uint16_t type, uint16_t value, uint32_t *data);
int xpci_modReadADC(unsigned modMask,float *VA,float *VD,float *VT,float *HV);

void 			  xpci_setLibStatus(unsigned status);
unsigned		  xpci_getLibStatus(void);

#ifdef __cplusplus
}
#endif

#endif

