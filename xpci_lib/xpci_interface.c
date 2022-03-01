/********************************************************************************
 *                               xpci_interface.c
 *
 * Library to use the PCIe board from PLDA with the XPIX customization.
 *
 * Communication are based on two communication channels numbered 0 and 1.
 * One channel encapsulates 3 low level FIFOS:
 * - One TX FIFO
 * - One RX FIFO
 * - One service FIFO to publish hardware error status on the channel operation.
 ********************************************************************************
 * PYD Creation 1/03/2010
 ********************************************************************************/
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <sys/time.h>
#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h>
#include <stdint.h>
#include <sched.h>
#include <signal.h>         // for sig_atomic_t


#include <pthread.h>        //for thread

//#include <semaphore.h>
//#include <time.h>

#include "plda_api.h"
//#include "plda_lib_access.h"
//#include "plda_ioctl.h"
//#include "plda_mod_registers.h"

#include <plda_ioctl.h>

// user interface
#include "xpci_interface.h"

// expert interface
#include "xpci_interface_expert.h"
#include "xpci_time.h"

// imxpad functions 
#include "xpci_imxpad.h"

/***********************************************************************************
// Constants definition
************************************************************************************/
#define NB_CHANNELS              2
#define NB_STATUS_REGS           16
#define NB_CMD_DEGS              12
#define SERV_BUFFER_SIZE         8    // Max size in bytes for hardware error status
#define TIMEOUT_CNT              2000 // given more or less in msec
#define TIMEOUT_HARD             29   // 1sec
#define LONGER_TIMEOUT_HARD      32   // 8sec
#define MAX_TX_FIFO_SIZE         2048 //2KBytes 2048 0x800
#define MOD_REPLY_SIZE           (16*2) // all replies have the same size 16words of 2 bytes
#define NB_WORDS_REPLY           0xD
#define MAX_MODULES_CHANNEL      4
#define MAX_DETECTOR_NB          1
#define CLOCK_PERIOD_NS          4    // in nanos

/* additional hardware timeouts */
#define HWTIMEOUT_DSBL           0  // applies only for IMXPAD systems
#define HWTIMEOUT_1SEC           29 // 2^(29-1)*4ns
#define HWTIMEOUT_2SEC           30 // 2^(30-1)*4ns
#define HWTIMEOUT_4SEC           31 // 2^(31-1)*4ns
#define HWTIMEOUT_8SEC           32 // 2^(32-1)*4ns

/* image parameters */
#define NB_COLUMN_CHIP           80
#define NB_LINE_CHIP             120

/* number of the locks for the dma buffers */
/*#define DMA0_TX_BUF_LOCK         0
#define DMA0_RX_BUF_LOCK         1
#define DMA0_SVC_BUF_LOCK        2
#define DMA1_TX_BUF_LOCK         3
#define DMA1_RX_BUF_LOCK         4
#define DMA1_SVC_BUF_LOCK        5
*/
/* DMA configuration registers offsets for one channel */
#define DMA0_TX_PHYADD_OFFSET	 0
#define DMA1_TX_PHYADD_OFFSET	 1
#define DMA0_TX_SIZE_OFFSET	 2
#define DMA1_TX_SIZE_OFFSET	 3
#define DMA0_RX_PHYADD_OFFSET	 4
#define DMA1_RX_PHYADD_OFFSET	 5
#define DMA0_RX_SIZE_OFFSET	 6
#define DMA1_RX_SIZE_OFFSET	 7
#define DMA0_SRV_PHYADD_OFFSET	 8
#define DMA1_SRV_PHYADD_OFFSET	 9
#define DMA0_CTRL		 10
#define DMA1_CTRL		 11
#define TIMEOUT_DELAY_REG        12
#define SUBCHNL_REG              13 // imXPAD register
#define PCIE_CMD_OFFSET          14

/* Command bits for DMAs */
#define START_TXDMA              0x00000001
#define ABORT_TXDMA              0x00000002
#define RESET_TXFIFO             0x00000004
#define START_RXDMA              0x00000100
#define ABORT_RXDMA              0x00000200
#define RESET_RXFIFO             0x00000400

#define DO_LOOP                  0x00000001

/* imXPAD command bits for subchannel registers */
#define SUBCHNL_TRNUM            0x00010000
#define SUBCHNL_TRSIZE_0A        0x00020000
#define SUBCHNL_TRSIZE_0B        0x00040000
#define SUBCHNL_TRSIZE_1A        0x00080000
#define SUBCHNL_TRSIZE_1B        0x00100000
#define SUBCHNL_TRLOOP           0x00200000
#define SUBCHNL_UPDTREG_0        0x00400000
#define SUBCHNL_UPDTREG_1        0x00800000

/* DMA status registers offsets for one channel */
#define DMA0_TX_ADD              0
#define DMA0_TX_SIZE             1
#define DMA0_RX_ADD              2
#define DMA0_RX_SIZE             3
#define DMA1_TX_ADD              4
#define DMA1_TX_SIZE             5
#define DMA1_RX_ADD              6
#define DMA1_RX_SIZE             7
#define DMA0_SRV_ADD             8
#define DMA1_SRV_ADD             9
#define RX_FIFOS_STATUS          10
#define FIRWARE_CODE             11
#define DMA_IT_FLAGS             15

/* DMA STATUS CODE */
#define DMA_STATUS_OK            0
#define DMA_STATUS_SIZEERR       1
#define DMA_STATUS_TIMEOUT       2
#define DMA_STATUS_RXERR         3
#define DMA_STATUS_TXERR         4

/* messages tags */
#define HUB_HEADER               0xBB44
#define MOD_MESSAGE              0x3333
#define HUB_MESSAGE              0xCCCC
#define MOD_HEADER               0xAA55
#define MSG_TRAILER              0xF0F0

/* hub commands */
#define HUB_LOOPBCK_EN           0x0001
#define HUB_LOOPBCK_DIS          0x0002
#define MOD_NIOS_REBOOT          0x02FF
#define HUB_FIRMWARE             0x03FF
#define HUB_FIFO_RESET           0x05FF
#define REG_FIFO_RESET		 0x5FF0
#define REG_MOD_NAME		 0x7FF0

/* modules commands */
#define READ_IMAGE_16b           0x0182
#define READ_IMAGE_32b           0x0183
#define MOD_READ_CONFIG          0x01C0
#define CONFIG_CHIP              0x0203
#define CONFIG_CHIP_ACK          0x1203            
#define MOD_REQ_READY            0x0101
#define MOD_REQ_READY_ACK        0x1101
#define MOD_REQ_ATEST            0x0102
#define MOD_REQ_ATEST_ACK        0x1102
#define MOD_REQ_CONFIG           0x0103
#define MOD_REQ_CONFIG_ACK       0x1103
#define MOD_REQ_FLATCFG          0x0104
#define MOD_REQ_FLATCFG_ACK      0x1104
#define MOD_SAVE_CONFIGL         0x0380
#define MOD_SAVE_CONFIGL_ACK     0x1380
#define MOD_SAVE_CONFIGG         0x0381
#define MOD_SAVE_CONFIGG_ACK     0x1381
#define MOD_LOAD_CONFIG          0x0480
#define MOD_LOAD_CONFIG_ACK      0x1480
#define MOD_EXPOSE               0x0140
#define MOD_EXPOSE_DDR		     0x0141   //added by fred
#define MOD_READ_TEMP_SENSOR     0x0108   //added by fred
#define MOD_READ_TEMP_SENSOR_ACK 0x1108   //added by fred
#define MOD_CMD_READ_ADC		 0x0500   //added by fred
#define MOD_CMD_READ_ADC_ACK	 0x1500   //added by fred
#define CMD_SET_HV				 0x0510   //added by fred
#define CMD_SET_HV_ACK			 0x0510   //added by fred
#define MOD_EXPOSE_ACK           0x1140
#define MOD_GETIMG2B             0x0141
#define MOD_GETIMG4B             0x0142
#define MOD_PULSE_FLAT	         0x0150
#define MOD_PULSE_FLAT_ACK       0x1150
#define MOD_PULSE_CONFIG         0x0151
#define MOD_PULSE_CONFIG_ACK     0x1151
#define MOD_EXPOSURE_PARAM       0x0190
#define MOD_EXPOSURE_PARAM_ACK   0x1190
#define MOD_READ_CONFIGG         0x0105
#define MOD_READ_CONFIGG_ACK     0x1105
#define MOD_PULSER_IMXPAD        0x0106
#define MOD_PULSER_IMXPAD_ACK    0x1106
#define MOD_IPI		         0x0160
#define MOD_IPI_ACK	         0x1160
#define MOD_IPI_PARAM	         0x0161
#define MOD_IPI_PARAM_ACK        0x1161
#define MOD_SAVE_EXPWAITTIMES    0x0170
#define MOD_SAVE_EXPWAITTIMES_ACK 0x1170

#define MOD_MEM_DIAG			 0x0520  // added by fred

/********************************************************************************
                        MACRO commands definitions
*********************************************************************************/

// get the channel number associated to a module
// NO they are inverted !!! #define CHANNEL(moduleId) (moduleId<=3) ? 0:1



#define CHANNEL(moduleId) (moduleId<=3) ? 1:0
#define CHANNEL_S1400(moduleId) (moduleId<=9) ? 1:0
#define CHANNEL_S700(moduleId) (moduleId<=4) ? 1:0

// Conditional use of waiting IT functions with or without timeout
// dmaStatus result of the IT interception
// timeout   max time to wait for the IT occurence
#define WAIT_IT(dmaStatus,timeout)  if(timeout) dmaStatus=timedWaitIT( timeout );else dmaStatus=waitIT();

/* STATUS of a board or detector */
enum DetectorStatus{DOWN, UP};
/* Type of hardware device */
enum DEVICES   {HUB_DEV, PCI_DEV};

enum COLOR { GREEN=32, RED=31, BLUE=34};

/***********************************************************************************
//             Structures definition
***********************************************************************************/
struct StatusRegTable {
    unsigned long entry[NB_STATUS_REGS];
};
struct ModuleReplyBuffer {
    unsigned long data[MOD_REPLY_SIZE];
};


/***********************************************************************************
             Known commands should be rounded to 8 bytes with padding
***********************************************************************************/


/*============================ HUB commands ======================================*/

uint16_t  HUB_loop[8]             = { 0, 0, HUB_HEADER, HUB_MESSAGE, 0x0003, 
                                      HUB_LOOPBCK_EN,// 0X0001
                                      0X0000,
                                      MSG_TRAILER };
uint16_t  HUB_noLoop[8]           = { 0, 0, HUB_HEADER, HUB_MESSAGE, 0x0003, 
                                      HUB_LOOPBCK_DIS,// 0X0002
                                      0X0000,
                                      MSG_TRAILER };
uint16_t  HUB_2bImgMode[8]        = { 0, 0, HUB_HEADER, HUB_MESSAGE, 0x0003, 
                                      READ_IMAGE_16b,// 0X0182
                                      0X0000,
                                      MSG_TRAILER };
uint16_t  HUB_4bImgMode[8]        = { 0, 0, HUB_HEADER, HUB_MESSAGE, 0x0003, 
                                      READ_IMAGE_32b,// 0X0183,
                                      0X0000,
                                      MSG_TRAILER };
uint16_t  HUB_rebootModNIOS[8]    = { 0, 0, HUB_HEADER, HUB_MESSAGE, 0x0003, 
                                      MOD_NIOS_REBOOT, // 0X02FF
                                      0x00FF,  // mask of the modules to reset
                                      MSG_TRAILER };      // write here (index 6) the module mask
uint16_t  HUB_reconfModFPGA[8]    = { 0, 0, HUB_HEADER, HUB_MESSAGE, 0x0003, 
                                      HUB_FIRMWARE, // 0X03FF,  reload fpga
                                      0x0021,       // write here the module mask
                                      MSG_TRAILER };
uint16_t  HUB_rst[8]              = { 0, 0, HUB_HEADER, HUB_MESSAGE, 0x0003, 
                                      HUB_FIFO_RESET, // 0X05FF,
                                      0X0000,
                                      MSG_TRAILER };
uint16_t  HUB_dummy[8]            = { 0, 0, HUB_HEADER, HUB_MESSAGE, 0x0003, 
                                      0Xaabb, 0Xccdd,// HUB dummy msg
                                      MSG_TRAILER };

uint16_t  REG_FIFO_rst[8]              = { 0, 0, HUB_HEADER, HUB_MESSAGE, 0x0003, 
                                           REG_FIFO_RESET, // 0X5FF0,
                                           0X0000,
                                           MSG_TRAILER };

uint16_t  REG_Module_name[8]          = { 0, 0, HUB_HEADER, HUB_MESSAGE, 0x0003, 
                                          REG_MOD_NAME, // 0X7FF0,
                                          0X0000,
                                          MSG_TRAILER };
/*============================ MODULE commands ===================================*/

uint16_t  MOD_askReady[16]        = { HUB_HEADER, MOD_MESSAGE, 
                                      0x0006, //nb words
                                      0x0021, // modules masks
                                      MOD_HEADER,
                                      0x0003, //nb words

                                      0,      // reserved
                                      MOD_REQ_READY, // 0x0101, ask_ready
                                      MSG_TRAILER,

                                      0,0,0,0,0,0,0};

uint16_t  MOD_askReadyRep[16]    = {  MOD_HEADER, 
                                      0x0003, //module nb from 1 to 8, Can't be null
                                      NB_WORDS_REPLY,//nb of following words always 16 for detector
                                      MOD_REQ_READY_ACK, // 0x1101, ask_ready_rep
                                      0,0,0,0,0,0,0,0,0,0,0, // 11 words for empty data
                                      MSG_TRAILER};

uint16_t  MOD_autoTest[16]        = { HUB_HEADER, MOD_MESSAGE, 
                                      0x0008, //nb words
                                      0x0021, // modules masks
                                      MOD_HEADER,
                                      0x0005, //nb words

                                      0,      // reserved
                                      MOD_REQ_ATEST, // 0x0102, auto_test
                                      0xface, // value to load in the detector
                                      0,      // test_mode
                                      MSG_TRAILER,

                                      0, 0, 0, 0, 0};	// digital test

uint16_t  MOD_configG[16]         = { HUB_HEADER, MOD_MESSAGE, 
                                      0x000C, //nb words
                                      0x0021, // modules masks
                                      MOD_HEADER,
                                      0x0009, //nb words

                                      0,      // reserved
                                      MOD_REQ_CONFIG, //0x0103 configG
                                      0x000f, // chipmask
                                      0,  //register id code
                                      52, //register value example
                                      0, 0, 0,
                                      MSG_TRAILER,

                                      0};

uint16_t  MOD_allConfigG[24]      = { HUB_HEADER, MOD_MESSAGE, 
                                      0x0012, //nb words
                                      0x0021, // modules masks
                                      MOD_HEADER,
                                      0x000e, //nb words
                                      0,       // reserved,
                                      CONFIG_CHIP, //0x0203, allConfigG
                                      0x000f, // chipmask
                                      0,  // CMOS_DSBL
                                      0,  // AMP_TP,
                                      0,  // ITHH,
                                      0,  // VADJ,
                                      0,  // VREF,
                                      52, // IMFP,
                                      40, // IOTA,
                                      60, // IPRE,
                                      24, // ITHL,
                                      145,// ITUNE,
                                      0,  // IBUFFER
                                      MSG_TRAILER,

                                      0, 0, 0};

uint16_t  MOD_flatConfig[16]      = { HUB_HEADER, MOD_MESSAGE, 
                                      0x0009, //nb words
                                      0x0021, // modules masks
                                      MOD_HEADER,
                                      0x0006, //nb words

                                      0,      // reserved
                                      MOD_REQ_FLATCFG, // 0x0104 flatConfig
                                      0,      // unused
                                      0x0000, // chipmask All the chips
                                      0x02, //value example
                                      MSG_TRAILER,

                                      0, 0, 0, 0};

uint16_t  MOD_img2B[16]           = { HUB_HEADER, MOD_MESSAGE, 
                                      0x0006, //nb words
                                      0x0021, // modules masks
                                      MOD_HEADER,
                                      0x0003, //nb words

                                      0,     // reserved
                                      READ_IMAGE_16b, // 0x0182 request img 2B
                                      MSG_TRAILER,

                                      0, 0, 0, 0, 0, 0, 0};



uint16_t  MOD_img4B[16]           = { HUB_HEADER, MOD_MESSAGE, 
                                      0x0006, //nb words
                                      0x0021, // modules masks
                                      MOD_HEADER,
                                      0x0003, //nb words

                                      0,      // reserved
                                      READ_IMAGE_32b, // 0x0183 request img 4B
                                      MSG_TRAILER,

                                      0, 0, 0, 0, 0, 0, 0};
                                      
uint16_t  MOD_memDiag[16]           = { HUB_HEADER, MOD_MESSAGE, 
                                      0x0008, //nb words
                                      0x0021, // modules masks
                                      MOD_HEADER,
                                      0x0005, //nb words

                                      0,      // reserved
                                      MOD_MEM_DIAG, // 0x0520 
                                      0,      // type
                                      0,      //value
                                      MSG_TRAILER,

                                      0, 0, 0, 0, 0};

uint16_t  MOD_readConfig[16]      = { HUB_HEADER, MOD_MESSAGE, 
                                      0x0006, //nb words
                                      0x0021, // modules masks
                                      MOD_HEADER,
                                      0x0003, //nb words

                                      0,     // reserved
                                      MOD_READ_CONFIG, // 0x01C0, readConfig
                                      MSG_TRAILER,

                                      0, 0, 0, 0, 0, 0, 0};

uint16_t  MOD_saveConfigL[96]     = { HUB_HEADER, MOD_MESSAGE, 
                                      0x0059, //nb words
                                      0x0021, // modules masks
                                      MOD_HEADER,
                                      0x0056, //nb words

                                      0,      // reserved
                                      MOD_SAVE_CONFIGL, // 0x0380, saveConfig
                                      0, // calib id
                                      0, //current chip
                                      0, // current row
                                      // 80 values
                                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                      MSG_TRAILER,

                                      0 , 0, 0, 0};

uint16_t  MOD_saveConfigG[24]     = { HUB_HEADER, MOD_MESSAGE, 
                                      0x000f, //nb words
                                      0x0021, // modules masks
                                      MOD_HEADER,
                                      0x000c, //nb words

                                      0,     // reserved
                                      MOD_SAVE_CONFIGG, // 0x0381, readConfig
                                      0, // calib id
                                      0, // register
                                      0,0,0,0,0,0,0, // values [2:8]
                                      MSG_TRAILER,

                                      0, 0, 0, 0, 0, 0};

uint16_t  MOD_detLoadConfig[16]   = { HUB_HEADER, MOD_MESSAGE, 
                                      0x0007, //nb words
                                      0x0021, // modules masks
                                      MOD_HEADER,
                                      0x0004, //nb words

                                      0,      // reserved
                                      MOD_LOAD_CONFIG, // 0x0480 detector load config
                                      0, //calib id
                                      MSG_TRAILER,

                                      0, 0, 0, 0, 0, 0};

uint16_t  MOD_expose[16]          = { HUB_HEADER, MOD_MESSAGE, 
                                      0x0009, //nb words
                                      0x0021, // modules masks
                                      MOD_HEADER,
                                      0x0006, //nb words

                                      0,      // reserved
                                      MOD_EXPOSE, // 0x0140
                                      0, // gate mode
                                      0, // gate length
                                      0, // time unit
                                      MSG_TRAILER,

                                      0, 0, 0, 0};

uint16_t  MOD_getImg2B[16]        = { HUB_HEADER, MOD_MESSAGE, 
                                      0x0009, //nb words
                                      0x0021, // modules masks
                                      MOD_HEADER,
                                      0x0006, //nb words

                                      0,      // reserved
                                      MOD_GETIMG2B, // 0x0141
                                      0, // gate mode
                                      0, // gate length
                                      0, // time unit
                                      MSG_TRAILER,

                                      0, 0, 0, 0};

uint16_t  MOD_getImg2B_XPAD32[16] = { HUB_HEADER, MOD_MESSAGE, 
                                      0x000A, //nb words
                                      0x0021, // modules masks
                                      MOD_HEADER,
                                      0x0007, //nb words

                                      0,      // reserved
                                      MOD_GETIMG2B, // 0x0141
                                      0, // gate mode
                                      0, // gate length
                                      0, // time unit
                                      0, // nloop
                                      MSG_TRAILER,

                                      0, 0, 0};

uint16_t  MOD_getImg4B[16]        = { HUB_HEADER, MOD_MESSAGE, 
                                      0x0009, //nb words
                                      0x0021, // modules masks
                                      MOD_HEADER,
                                      0x0006, //nb words

                                      0,      // reserved
                                      MOD_GETIMG4B, // 0x0142
                                      0, // gate mode
                                      0, // gate length
                                      0, // time unit
                                      MSG_TRAILER,

                                      0, 0, 0, 0};


/*============================ Internal Pulser commands  =========================*/

uint16_t  MOD_PulseFlat[16]       = { HUB_HEADER, MOD_MESSAGE, 
                                      0x000A, //nb words
                                      0x0021, // modules masks
                                      MOD_HEADER,
                                      0x0007, //nb words

                                      0,      // reserved
                                      MOD_PULSE_FLAT, // 0x0150
                                      0,      // chipmask All the chips
                                      0x0000, // Value DACL
                                      0x00,   // Value ampl pulse
                                      0x00,   // NB hit
                                      MSG_TRAILER,

                                      0, 0, 0};

uint16_t  MOD_PulseConfig[16]     = { HUB_HEADER, MOD_MESSAGE, 
                                      0x0009, //nb words
                                      0x0021, // modules masks
                                      MOD_HEADER,
                                      0x0007, //nb words

                                      0,      // reserved
                                      MOD_PULSE_CONFIG, // 0x0151
                                      0,      // chipmask All the chips
                                      0x0000, // calib ID
                                      0x00,   // Value ampl pulse
                                      0x00,   //Nb hit
                                      MSG_TRAILER,

                                      0, 0, 0};


/*============================ imXPAD commands ===================================*/

uint16_t  MOD_exposureParam[32]   = { HUB_HEADER, MOD_MESSAGE, 
                                      0x001B, //nb words
                                      0x0000, // modules masks
                                      MOD_HEADER,
                                      0x0018, //nb words

                                      0,    // reserved
                                      MOD_EXPOSURE_PARAM, // exposure parameters command
                                      0x0,    // exposure time H
                                      0x0,    // exposure time L
                                      0x0,    // dead time H
                                      0x0,    // dead time L
                                      0x0,    // init time H
                                      0x0,    // init time L
                                      0x0,    // shutter time H
                                      0x0,    // shutter time L
                                      0x0,    // ovf scan period H
                                      0x0,    // ovf scan period L
                                      0x0,    // exposure mode
                                      0x0,    // N param
                                      0x0,    // P param
                                      0x0,    // Number of images
                                      0x0,    // Busy out select
                                      0x0,    // Image data format
                                      0x0,    // Post-processing enabled
                                      0x0,    // GP 0
                                      0x0,    // GP 1
                                      0x0,    // GP 2
                                      0x0,    // GP 3
                                      MSG_TRAILER,

                                      0, 0};
uint16_t  MOD_saveWaitExpTimes[112]= { HUB_HEADER, MOD_MESSAGE, 
                                       0x006C, //nb words
                                       0x0021, // modules masks
                                       MOD_HEADER,
                                       0x0069, //nb words

                                       0,      // reserved
                                       MOD_SAVE_EXPWAITTIMES, // 0x0380,
                                       0, // start value
                                       0, // end value
                                       // 50 values (low and high half)
                                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                       MSG_TRAILER,

                                       0};

uint16_t  MOD_readConfigG[16]     = { HUB_HEADER, MOD_MESSAGE, 
                                      0x000C, //nb words
                                      0x0021, // modules masks
                                      MOD_HEADER,
                                      0x0009, //nb words

                                      0,      // reserved
                                      MOD_READ_CONFIGG, //0x0105, readConfigG
                                      0x000f, // chipmask
                                      0,  //register id code
                                      0,
                                      0, 0, 0,
                                      MSG_TRAILER,

                                      0};

uint16_t  MOD_PulserImxpad[16]    = { HUB_HEADER, MOD_MESSAGE, 
                                      0x0008, //nb words
                                      0x0021, // modules masks
                                      MOD_HEADER,
                                      0x0005, //nb words

                                      0,     // reserved
                                      MOD_PULSER_IMXPAD, // 0x0106
                                      0, // pulses
                                      0, // mask
                                      MSG_TRAILER,

                                      0, 0, 0, 0, 0};

uint16_t  MOD_sendIPI[16]         = { HUB_HEADER, MOD_MESSAGE, 
                                      0x0008, //nb words
                                      0x0021, // modules masks
                                      MOD_HEADER,
                                      0x0005, //nb words

                                      0,     // reserved
                                      MOD_IPI, // 0x0150
                                      0, // iterations
                                      0, //
                                      MSG_TRAILER,

                                      0, 0, 0, 0, 0};

uint16_t  MOD_sendIPIParam[16]    = { HUB_HEADER, MOD_MESSAGE, 
                                      0x0008, //nb words
                                      0x0021, // modules masks
                                      MOD_HEADER,
                                      0x0005, //nb words

                                      0,     // reserved
                                      MOD_IPI_PARAM, // 0x0150
                                      0, // enable IPI reading
                                      0, // image nr
                                      MSG_TRAILER,

                                      0, 0, 0, 0, 0};
                                      
uint16_t  MOD_TempSensor[16]      = { HUB_HEADER, MOD_MESSAGE, 
                                      0x0006, //nb words
                                      0x0021, // modules masks
                                      MOD_HEADER,
                                      0x0003, //nb words

                                      0,     // reserved
                                      MOD_READ_TEMP_SENSOR, // 0x0108, read temps sensor
                                      MSG_TRAILER,

                                      0, 0, 0, 0, 0, 0, 0};      
                                      
uint16_t MOD_readADC[16]        = { HUB_HEADER, MOD_MESSAGE,
                                     0x0006,       //nb words
                                     0x0021,      //modules masks
                                     MOD_HEADER,
                                     0x0003,       //nb words
                                     0,            //reserved
                                     MOD_CMD_READ_ADC, //0x0500, Read ADC ( Channel_0 : HV, Channel_1 : VA, Channel_2 : VD, Channel_3 : VT )
                                     MSG_TRAILER,
                                     0,0,0,0,0,0,0};    
                                     
uint16_t  MOD_setHV[16]        = { HUB_HEADER, MOD_MESSAGE,
									 0x0007,       //nb words
									 0x0021,      //modules masks
									 MOD_HEADER,
									 0x0004,       //nb words

									 0,            //reserved
									 CMD_SET_HV, //0x0510,
									 160,
									 MSG_TRAILER,

									 0,0,0,0,0,0};                            
                                      
                                      
                                      

uint16_t  MOD_imxpadRstNIOS[16]   = { HUB_HEADER, HUB_MESSAGE, 0x0002,
                                      0x0001, // bit(0) - reset NIOS
                                      MSG_TRAILER,
                                      0, 0, 0};

uint16_t  MOD_imxpadAbortExp[16]  = { HUB_HEADER, HUB_MESSAGE, 0x0002,
                                      0x0002, // bit(1) - abort exposure
                                      MSG_TRAILER,
                                      0,0,0};

/**********************************************************************************/
//                                  Globals 
/**********************************************************************************/
//                              GENERAl VARIABLES
/**********************************************************************************/
int                             xpci_systemType = 0; // describe the architecture
int                             xpci_systemChip = 0; // describe the kind of chip
static int                      debugMsg = 0;
static int                      nIndex   = 0; // identify a given plda board
volatile sig_atomic_t           it_cnt   = 0;
int                             it_total = 0;
pthread_mutex_t	                mutex_dma = PTHREAD_MUTEX_INITIALIZER;
static int                      xpci_it_pos  = 0;
static unsigned                 spy_mem_add;
static char                     *error_msg_channel[NB_CHANNELS];
static enum DetectorStatus      detStatus[MAX_DETECTOR_NB];
static SPCISlotDescription      PCIslotDescription;
static StatusRegTable           lastRegs; /*last read values for status regs*/
static SBufferDescription       svcDma0, svcDma1;

char                            dum; // to prompt user to go on

// bit mask of enable image post processings
// bit [0] enable geometrical rearangement
// bit [1] dead pixels correction
unsigned                        imxpad_postProc = 0; 


//********************************************************************************
//                        GLOBALS FOR IMAGE READING
//
// Used to keep configuration and parameters for a bunch of images acquisition not
// to reallocate them at each image acquisition. This speeds up the process.
//
//*********************************************************************************
static SBufferDescription       img_rdBuffer0, img_rdBuffer1;
static SBufferDescription       img_txBuffer0, img_txBuffer1;

/* number of the locks for the dma buffers */
unsigned DMA0_TX_BUF_LOCK ; //0
unsigned DMA0_RX_BUF_LOCK ; //1
unsigned DMA0_SVC_BUF_LOCK; //2
unsigned DMA1_TX_BUF_LOCK ; //3
unsigned DMA1_RX_BUF_LOCK ; //4
unsigned DMA1_SVC_BUF_LOCK; //5

static int                      img_nbMod0, img_nbMod1;        // nb modules to read per channel
static int                      img_nbParallelTrans, img_nbSeqTrans;
static int                      img_moduleMask;
//static enum IMG_TYPE           img_type; // 2 or 4 Bytes
enum IMG_TYPE                    img_type; // 2 or 4 Bytes
static int                      img_transferSize;
static int                      img_sizeImage;
static int                      img_gateMode, img_gateLength, img_timeUnit;
// used to know if we do readNext with or without expose before
static int                      img_expose = 0; // default is read without automatic expose
static int                      img_gotImages = 0; // images already got
static int                      detectorBusy   = 0;

static int                      mode12bits = 0; // use for S1400 ALBA

unsigned int 					 img_Format_Acq;
unsigned int 					 flag_startExpose = 0;


static int 					 lib_status=0;

int                             acquisition_type;                   //Used to indicate Normal, Burst or Film Mode
//    0-Standard Mode ( 16 or 32 bits 400 images max )
//    1-Burst Mode ( 16 bits 900 images max )
//    2-Film Mode ( 16 bits )
// 	  Used for GetImgSeq_SSD_imxpad
uint16_t                 **pRawBuff_ssd;  // used for img seq film
int                         read_pRawBuff_ssd=0;
int                         write_pRawBuff_ssd=0;
int						    AbortProccess = 0;
int 						ResetProcess  = 0;

unsigned int 				FirmwareID = 0;

//**********************************************************************************
//                         IT MANAGEMENT DATA
//**********************************************************************************
static SPCIInterruptTransfer    RefDesignInterruptInit =
{
    ///*  	Access	Bar		BYTE		Reset */
    ///*	Type		Index		offset	Value */
    // ISR_RW,	BAR_0,	0x3C,		0xffffffff,
    ISR_RW,		BAR_0,		0x3C,	0xffffffff,		1,			PIN_BASED_INT, 	1,
};






/**********************************************************************************
             Expert oriented functions internal to this library
***********************************************************************************/
//static void     interruptHandler(void);
static void     interruptHandler(SPCIInterruptData*  pData);
static int      timedWaitOnCond(int timeout , int channel, int (*testFunc)(int para));
static int      waitIT();
static int      timedWaitIT( int timeout );
static int      xpci_waitOnChannel(int channel, int softTimeout);
static int      xpci_allocateDmaMemory(unsigned int *lockNb, SBufferDescription *bufDesc, int size);
static void     xpci_eraseSvcMsg(int channel);
static int      roundSizeToXBytes(int size, int nbBytes);
static int      xpci_writeSplitted(int channel, uint16_t *data, int size);
static int      xpci_writeExec(int channel, uint16_t *data, int size, int noReset);
static int      xpci_doCommand(int channel, unsigned cmd);
static int      xpci_resetHardware(int det);
static int      xpci_resetReadFifo(int channel);
static int      xpci_resetWriteFifo(int channel);
static int      xpci_abortRead(int channel);
static int      xpci_abortWrite(int channel);


/**********************************************************************************/
//                   Functions 
/**********************************************************************************/
/*******************************************
 *  Print commodity
 *******************************************/
static void cprintf(char *str, enum COLOR cr)
{
    printf("%c[%d;%dm%s%c[%dm",27,1,cr,str, 27,0);

}
/*******************************************
 * misceallenous commodities functions
 *******************************************/
// Activate-deactivate debug mode
//====================xpci_allocateDmaMemory===========
void xpci_debugMsg(int flag){
    debugMsg = flag;
}


unsigned xpci_getLibStatus(void){
	return lib_status;
}

void xpci_setLibStatus(unsigned status){
	lib_status = status;
}

/*********************************************************/
/*!\brief
 * Returns 1(true) if at least one module is in use on the channel chnl
 *
 * Details: ATTENTION logic is inverted low channel is high modules!!!
 */
/*********************************************************/
int isChannelUsed(int chnl, unsigned modMask){
    // ATTENTION logic is inverted low channel is high modules!!!
    if(xpci_systemType==IMXPAD_S1400)
    {
        if (chnl==1) return modMask & 0x3ff;
        else if (chnl==0) return modMask & 0xffc00;
        else return modMask & 0xfffff;
    }    
    else if(xpci_systemType==IMXPAD_S700)
    {
        if (chnl==1) return modMask & 0x1f;
        else if (chnl==0) return modMask & 0x3E0;
        else return modMask & 0x3ff;
    }

    if (chnl==1) return modMask & 0xf;
    else if (chnl==0) return modMask & 0xf0;
    else return modMask & 0xff;
}
/*********************************************************/
/*!\brief
 * Returns the number of modules connected to one channel
 */
/*********************************************************/
int nbModOnChannel(int chnl, unsigned modMask){
    int i;
    int nb=0;
    // ATTENTION logic is inverted low channel is high modules!!!
    
    if(xpci_systemType==IMXPAD_S1400){
        if (chnl==0)
            modMask = modMask>>10;

        for (i=0; i<10; i++)
            if (((modMask>>i)&0x1)==1)
                nb++;
        return nb;
    }
    else if(xpci_systemType==IMXPAD_S700){
        if (chnl==0)
            modMask = modMask>>5;
        for (i=0; i<5; i++)
            if (((modMask>>i)&0x1)==1)
                nb++;
        return nb;
    }   
    
    if (chnl==0)
        modMask = modMask>>4;
    for (i=0; i<4; i++)
        if (((modMask>>i)&0x1)==1)
            nb++;
    return nb;
}
// Function to read the nb of bytes in RX FIFOs
// return : the total in all Fifos
// out    : the level in fifo 0 and fifo 1
//===========================================================
static int   xpci_getFifoFillLevel(int *rx0, int *rx1, int print){
    unsigned regVal;
    PldaMemoryRead32(nIndex, BAR_0, 0, NB_STATUS_REGS, lastRegs.entry);
    regVal = ((unsigned)lastRegs.entry[10]);
    *rx1 = regVal >> 16;
    *rx0 = regVal & 0xff;
    if (print)
        if(regVal != 0)
        {
            printf("%s: Fifo 0 = %d bytes, Fifo 1 = %d\n",__func__,*rx0, *rx1);
        }
    return *rx0+*rx1;
}
// Function to test the fill level of the DMA FIFO
// returns : 0 fifo empty, >0 fifo contains data
// in      : channel 0, 1 , >1 any fifos
//=================================================
static int xpci_testFifoLevel(int target){
    int total, rx0, rx1;;
    total = xpci_getFifoFillLevel(&rx0, &rx1, 0);
    if      (target==0)      return rx0;
    else if (target==1)      return rx1;
    else                     return total;
}
// Function to wait that some data are received in the RX fifo
// return  : 0 success data available, -1 timeout error
// in      : channel    0, 1 , >1 means any channel
//         : timeout    maximum delay to wait for those data
//================================================================
int xpci_waitDataInFifo(int channel, int timeout){
    return timedWaitOnCond(timeout , channel, xpci_testFifoLevel);
}
/****************************************************
 *                   global protections
 *****************************************************/
static int xpci_isBusyDetector(){
    return detectorBusy?0:1;
}
static int xpci_takeDetector(){
    if  (!detectorBusy) {
        detectorBusy=1;
        printf("Take detector\n");
        return  0;
    }
    else {
        printf("ERROR: detector busy reading\n");
        return -1;
    }
}
static void xpci_freeDetector(){
    detectorBusy=0;
    printf("Free detector\n");
}

unsigned int getFirmwareId(void)
{
	return FirmwareID;
}

// Activate-Abort mode
//===============================
void xpci_setAbortProcess(){
   // printf("=>>>>\t%s\n",__func__);
    AbortProccess = 1;
}

void xpci_clearAbortProcess(){
    AbortProccess = 0;
}


// Activate-Reset mode
//===============================
void xpci_setResetProcess(){
   // printf("=>>>>\t%s\n",__func__);
    ResetProcess = 1;
}

void xpci_clearResetProcess(){
    ResetProcess = 0;
}


void xpci_AbortCleanDetector(unsigned modMask)
{
    uint16_t *msg;
    
    for (int i=0; i<2; i++){
		if (xpci_systemType == IMXPAD_S1400 || xpci_systemType == IMXPAD_S700){
			xpci_regRstFifo();
		}

		//xpix_imxpadWriteSubchnlReg(modMask, 0, 1);
		xpix_imxpadWriteSubchnlReg(0, 0, 1);
		// send Ask Ready command
		msg = malloc(sizeof(MOD_askReady));
		memcpy(msg,MOD_askReady,sizeof(MOD_askReady));
		msg[3] = modMask; // set the mask value
		//  ret = xpci_writeCommon(msg, sizeof(MOD_askReady));
		if(xpci_systemType == IMXPAD_S1400){
			xpci_writeCommon_S1400(msg, sizeof(MOD_askReady),modMask);
		}
		else{
			xpci_writeCommon(msg, sizeof(MOD_askReady));
		}
		usleep(10);
		if(xpci_systemType == IMXPAD_S1400){
			xpci_writeCommon_S1400(msg, sizeof(MOD_askReady),modMask);
		}
		else{
			xpci_writeCommon(msg, sizeof(MOD_askReady));
		}
		free(msg);
	}
}

int xpci_getAbortProcess(){
    return AbortProccess;
}

int xpci_getResetProcess(){
    return ResetProcess;
}
/****************************************************
 *    imXPAD subchannel register configuration
 ***************************************************/
/******************************************************************************
 * Function configures PCIe registers to control data flow on the subchannel
 * in direction from the detector to the PC. The system contains 2 subchannels per
 * channel (4 in total) and each of the subchannel receives a data from 2 detector
 * modules. The subchannels writes data to obn of two RXFIFO availiable in the
 * system (one per channel)
 * Subchannel registers:
 * - subchnl_trnum_0a - 4 bits, number of transfers on the subchannel 1a
 * - subchnl_trnum_0b - 4 bits, number of transfers on the subchannel 1b
 * - subchnl_trnum_1a - 4 bits, number of transfers on the subchannel 2a
 * - subchnl_trnum_1b - 4 bits, number of transfers on the subchannel 2b
 * - subchnl_trsize_0a - 16 bits, size of the transfer on the subchannel 1a
 * - subchnl_trsize_0a - 16 bits, size of the transfer on the subchannel 1b
 * - subchnl_trsize_1a - 16 bits, size of the transfer on the subchannel 2a
 * - subchnl_trsize_1b - 16 bits, size of the transfer on the subchannel 2b
 * - subchnl_trloop - 16 bits, number of repetitive loops (used when reading images)
 * - subchnl_updt_0 - 1 bit, indicates that nez registers are written (channel 0)
 * - subchnl_updt_1 - 1 bit, indicates that nez registers are written (channel 1)
 * In order to write 9 above registers, the plda register with offset 13 is used
 * The upper half of the plda register (bits[31:16]) is used to encode which
 * of the subchannel register has to be configured while lower half of the plda
 * register contains a register value.
 ******************************************************************************/
int xpix_imxpadWriteSubchnlReg(unsigned modMask, unsigned msgType, unsigned trloops){

    unsigned modMaskSubchnl_1a = 0x3 &  modMask;
    unsigned modMaskSubchnl_1b = 0x3 & (modMask>>2);
    unsigned modMaskSubchnl_0a = 0x3 & (modMask>>4);
    unsigned modMaskSubchnl_0b = 0x3 & (modMask>>6);
    
    // only selected system types
    if(xpci_systemType==IMXPAD_S1400){
        modMaskSubchnl_1a = (0x1F &  modMask);
        modMaskSubchnl_1b = (0x1F & (modMask>>5));
        modMaskSubchnl_0a = (0x1F & (modMask>>15));
        modMaskSubchnl_0b = (0x1F & (modMask>>10));
    //    xpci_regRstFifo();
    }        // only selected system types
    else if(xpci_systemType==IMXPAD_S700){
        modMaskSubchnl_1a = (0x1F &  modMask);
        modMaskSubchnl_1b =  0x00;
        modMaskSubchnl_0a = (0x1F & (modMask>>5));
        modMaskSubchnl_0b =  0x00;
    //    xpci_regRstFifo();
    }
    xpci_regRstFifo();
    unsigned modNbSubchnl_0a = xpci_getModNb(modMaskSubchnl_0a);
    unsigned modNbSubchnl_0b = xpci_getModNb(modMaskSubchnl_0b);
    unsigned modNbSubchnl_1a = xpci_getModNb(modMaskSubchnl_1a);
    unsigned modNbSubchnl_1b = xpci_getModNb(modMaskSubchnl_1b);
    unsigned regTrNum_0a = 0;
    unsigned regTrNum_0b = 0;
    unsigned regTrNum_1a = 0;
    unsigned regTrNum_1b = 0;
    unsigned regTrSize_0a = 0;
    unsigned regTrSize_0b = 0;
    unsigned regTrSize_1a = 0;
    unsigned regTrSize_1b = 0;
    unsigned long command = 0;
    
    // only selected system types
 //   if(xpci_systemType==IMXPAD_S540 || xpci_systemType==IMXPAD_S420
//				|| xpci_systemType==IMXPAD_S1400 || xpci_systemType==IMXPAD_S700){

        switch(msgType){
        // commands
        case 0:
            // number of tranfers
            if(modNbSubchnl_0a==0) regTrNum_0a = 0;
            else regTrNum_0a = 1;
            if(modNbSubchnl_0b==0) regTrNum_0b = 0;
            else regTrNum_0b = 1;
            if(modNbSubchnl_1a==0) regTrNum_1a = 0;
            else regTrNum_1a = 1;
            if(modNbSubchnl_1b==0) regTrNum_1b = 0;
            else regTrNum_1b = 1;
            // size of tranfers
            regTrSize_0a = modNbSubchnl_0a*16;
            regTrSize_0b = modNbSubchnl_0b*16;
            regTrSize_1a = modNbSubchnl_1a*16;
            regTrSize_1b = modNbSubchnl_1b*16;

            
            break;

            // 16b image
        case 1:
            // number of tranfers
            regTrNum_0a = 2*modNbSubchnl_0a;
            regTrNum_0b = 2*modNbSubchnl_0b;
            regTrNum_1a = 2*modNbSubchnl_1a;
            regTrNum_1b = 2*modNbSubchnl_1b;
            // size of tranfers
            if(modNbSubchnl_0a==0) regTrSize_0a = 0;
            else regTrSize_0a = 60*566;
            if(modNbSubchnl_0b==0) regTrSize_0b = 0;
            else regTrSize_0b = 60*566;
            if(modNbSubchnl_1a==0) regTrSize_1a = 0;
            else regTrSize_1a = 60*566;
            if(modNbSubchnl_1b==0) regTrSize_1b = 0;
            else regTrSize_1b = 60*566;
            break;

            // 32b image
        case 2:
            // number of tranfers
            regTrNum_0a = 3*modNbSubchnl_0a;
            regTrNum_0b = 3*modNbSubchnl_0b;
            regTrNum_1a = 3*modNbSubchnl_1a;
            regTrNum_1b = 3*modNbSubchnl_1b;
            // size of tranfers
            if(modNbSubchnl_0a==0) regTrSize_0a = 0;
            else regTrSize_0a = 40*1126;
            if(modNbSubchnl_0b==0) regTrSize_0b = 0;
            else regTrSize_0b = 40*1126;
            if(modNbSubchnl_1a==0) regTrSize_1a = 0;
            else regTrSize_1a = 40*1126;
            if(modNbSubchnl_1b==0) regTrSize_1b = 0;
            else regTrSize_1b = 40*1126;
            break;

        default:
            break;
        }

        // write register values
        // transfer numbers
        command = SUBCHNL_TRNUM + (regTrNum_1b<<12) + (regTrNum_1a<<8) + (regTrNum_0b<<4) + regTrNum_0a;
        //  printf("command  SUBCHNL_TRNUM = 0x%x\n",command);
        PldaMemoryWrite32 (nIndex, BAR_0, SUBCHNL_REG, 1, &command);
        // size 0a (divided by 4 because we 64 bits zords are read from subchannel FIFO)
        command = SUBCHNL_TRSIZE_0A + (regTrSize_0a/4);
        //  printf("command SUBCHNL_TRSIZE_0A = 0x%x\n",command);
        PldaMemoryWrite32 (nIndex, BAR_0, SUBCHNL_REG, 1, &command);
        // size 0b (divided by 4 because we 64 bits zords are read from subchannel FIFO)
        command = SUBCHNL_TRSIZE_0B + (regTrSize_0b/4);
        //  printf("command SUBCHNL_TRSIZE_0B = 0x%x\n",command);
        PldaMemoryWrite32 (nIndex, BAR_0, SUBCHNL_REG, 1, &command);
        // size 1a (divided by 4 because we 64 bits zords are read from subchannel FIFO)
        command = SUBCHNL_TRSIZE_1A + (regTrSize_1a/4);
        //  printf("command SUBCHNL_TRSIZE_1A = 0x%x\n",command);
        PldaMemoryWrite32 (nIndex, BAR_0, SUBCHNL_REG, 1, &command);
        // size 1b (divided by 4 because we 64 bits zords are read from subchannel FIFO)
        command = SUBCHNL_TRSIZE_1B + (regTrSize_1b/4);
        //  printf("command SUBCHNL_TRSIZE_1B = 0x%x\n",command);
        PldaMemoryWrite32 (nIndex, BAR_0, SUBCHNL_REG, 1, &command);
        // loops
        command = SUBCHNL_TRLOOP + trloops;
        //  printf("command SUBCHNL_TRLOOP = 0x%x\n",command);
        PldaMemoryWrite32 (nIndex, BAR_0, SUBCHNL_REG, 1, &command);
        // update 0
        // update 1
        command = SUBCHNL_UPDTREG_1 + SUBCHNL_UPDTREG_0;
        //  printf("command SUBCHNL_UPDTREG_1 = 0x%x\n",command);
        PldaMemoryWrite32 (nIndex, BAR_0, SUBCHNL_REG, 1, &command);
        
        command = SUBCHNL_UPDTREG_1 + SUBCHNL_UPDTREG_0;
        //  printf("command SUBCHNL_UPDTREG_1 = 0x%x\n",command);
        PldaMemoryWrite32 (nIndex, BAR_0, SUBCHNL_REG, 1, &command);

  //  }

    return 0;
}

/****************************************************
 *                   Interrupt handler
 *****************************************************/

void initIt(){
    //printf("---> IT flag reset \n");
    it_cnt = 0;// it_total = 0;
}
// The format needs a dum para to satisfy the  timedWaitOnCond() passed function signature
int xpci_getItCnt(int dum){
    return it_cnt;
}
int xpci_getTotalItCnt(){
    return it_total;
}
void xpci_itStatus(){
    printf("waiting position is %d\n", xpci_it_pos);
}
/*
static void interruptHandler(void)
{
    //it_cnt++;
    xpci_setItCount();
    // printf(     " **          interrupt received        **\n");
    // it_total++;
}
*/

static void interruptHandler(SPCIInterruptData*  pData)
{
    //it_cnt++;
  // printf("Interrupt status = 0x%x\n", pData->data);
   
    xpci_setItCount();
    // printf(     " **          interrupt received        **\n");
    // it_total++;
}

void xpci_setItCount()
{
    it_cnt++;
}

void xpci_clearItCount()
{
    it_cnt=0;
}

int xpci_getItCount()
{
    return it_cnt;
}

/****************************************************************
                  Design for semaphore use

    We will use a semaphore to unblock the main thread while it
waits for IT. Th semaphore is taken by the main at the xpci_init()
and then it tries to take it again for the first wait(). Then the
IT will unlock it unlocking the semaphore. then returning from
the wait() the main() will lock again the semaphore to be ready for
the next wait().
   This has been tested in a standalone programme
example_mutex_in_main() which works nevertheless the doc says
that the pthread_mutex_unlock() and most of those functions
are not safe in threads!!!
***************************************************************/
/*
static void interruptHandlerSem(void)
{
    it_cnt++;
    pthread_mutex_unlock(& mutex_dma);
}
//Function called to wait on it with a semaphore
void semWaitOnIT(){
  // first lock to have the mutex busy
pthread_mutex_lock(& mutex_dma);
  // second lock to be blocked if
  }*/
/***************************************************************
 * Set the hardware timeout value to the passed value.
 * The timer is a 32 bits register that is used with a not continuous
 * scale of 32 values. The time associated to each bit is based on a
 * 250MHz clock (4ns resolution).
 * The delay associated to bit x is: 4ns * (2 ** (x-1))
 * bit 1 = 4ns
 * bit 2 = 8ns
 * ...
 * bit 8  = 512ns
 * bit 9  = 1024ns    (1us)
 * bit 11 = 4096ns    (4us)
 * bit 12 = 8192ns    (8us)
 * bit 13 = 16384ns   (16us)
 * bit 14 = 32768ns   (32us)
 * bit 15 = 65536ns   (65us)
 * bit 16 = 131072    (131us)
 * bit 17 = 262144ns  (262us)
 * bit 19 = 1048576ns (1ms)
 * bit 20 =           (2ms)
 * bit 21 =           (4ms)
 * bit 22 =           (8ms)
 * bit 23 =           (16ms)
 * bit 24 =           (32ms)
 * bit 25 =           (64ms)
 * bit 26 =           (128ms)
 * bit 27 =           (256ms)
 * bit 28 =           (512ms)
 * bit 29 =           (1s)
 * bit 30 =           (2s)
 * bit 31 =           (4s)
 * bit 32 =           (8s)
 *************************************************************************/
void  xpci_setHardTimeout(int value){
    PldaMemoryWrite32 (nIndex, BAR_0,TIMEOUT_DELAY_REG, 1 , (DWORD*)(&value));
}

int   xpci_getHardTimeout(int print){
    int regVal, delay;
    PldaMemoryRead32(nIndex, BAR_0, 0, NB_STATUS_REGS, lastRegs.entry);
    regVal = ((unsigned)lastRegs.entry[12]) & 0xff;
    if (print){
        delay = CLOCK_PERIOD_NS * (1 << (regVal-1));
        printf("%s(): Hardware timer is set to %d nanos (register value %d)\n",__func__,delay, regVal);
    }
    return regVal;
}

/***************************************************************
* Function to wait with a coarse software timeout on a condition 
* provided by a function that that shoul return true when the condition 
* is satified and false as long as it is not.
* more or less in millisec
*
* 
* returns : 0 succes, -1 timeout
* in      : timeout   software timeout
*         : channel   0,1 or both if >1
*         : testFunc  function to call (int tesFunc(int para))
*
* Note on usage for DMA IT detection:
* An IT can appear for a proper transaction end or a hardware
* timeout end. The calling function should test the hardware
* service FIFO to know if it is a normal end or a bad end on
* timeout. The passed timeout to this function is a safty limit
* in case the IT would be lost for a reason or another. It should
* set to a value higher than the current hardware timer value
* (see xpci_setHardTimeout()) to be never reached.
****************************************************************/
// Tests have demonstrated that the software timeout para is more 
// or less evaluated in MILLISEC. A value 1000 gives a timeout 
// of 1,1 sec which means that 1000 usleep(1) gives 1000usec with
// about 30% instability.
static int timedWaitOnCond( int timeout , int channel, int (*testFunc)(int para)){ 
    int           delay = 0;
    int           ret, error;
    unsigned  long       command;

    //  xpci_timerStart(0);
    error = 0;
    /* This value in usleep() is not very significant in detail because
     a value 1 gives a delay from 100 to 200 one a value of 1000 for
     timeout with an instable measure. Only after pushing the value at
     50 we can detect a real change of about 40% on average. */

    while (testFunc(channel) == 0){
        if (delay < timeout){
            delay++;
            ret = usleep((unsigned long)1);
        }
        else {
            /* abort the pending transactions */
            xpci_resetChannels(3); // 3 for all channels
            command=0xffffffff;
            if (xpci_getInterruptStatus()==1)
                PldaMemoryWrite32 (nIndex,BAR_0, 15, 1 , &command);
            //   xpci_timerStop(0); // prints the delay
            error = -1;
            break;
        }
    }
    //xpci_timerStop(0); // prints the delay
    return error;
}
/***********************************************************************************
                               TIMEOUT MANAGEMENT ON DMA

Hardware timeout:

There is a hardware timeout that can be set with function 
xpci_setHardTimeout(val)
with a delay ranging from 4ns to 8sec depending of the 'val' value (see the function
description). 
When this value is reached an IT is generated and the dma status is set to 
DMA_STATUS_TIMEOUT.


Software timeout:

As we have experimented that sometimes the IT get lost it has been necessary to add
another software timeout to protect against this issue and prevent the process to
remain blocked for ever in a spin loop on the IT flag. Two functions are available
to wait with a protected software timeout. 

1- A fast one:
int waitIT()
adjusted to be used for burst image reading at high speed

2- A timed one based on usec delay to adjust larger timeout delay
int timedWaitIT( int timeout )

When the timeout is reached those function returns the value -1 instead of 0.

ATTENTION: if you want to use the software timeout adjust the hardware timeout to
a larger value!
**********************************************************************************/

/**********************************************************************************
imXPAD note:
Hardware timeout:
The imXPAD systems allows to disable the hardware timeout to allow the detector 
to send a data with a very long non determined delay.

Software timeout:
With a new PC hardware used by imXPAD (ASUS SABERTOOTH X58) and a new PLDA driver (v181) 
we do not experience IT losts anymore. Therefor, in the imXPAD systems we do not use any 
protection as describet above.
**********************************************************************************/


/***************************************************************
* Function to wait efficiently on on the IT. This is the fastest
* version to use for fast images acquisition.
* The timeout delay is a constant safety margin that has been 
* adjusted to prevent against IT loss. But it is a short
* timeout adapted for fast image reading.
* 
* ATTENTION: the delay should have a higher value that the
* hardware timeout or this function will always end with success. 
* But with a timeout error status.
* In any case the user should test the error status in the service 
* fifo to check for a hardware timeout error even if this function
* is successfull.
* 
* returns 0 succes 
*         -1 IT lost or IT handler not executed
****************************************************************/
static int waitIT(){
    int           delay = 0;
    int           error = 0;
    UINT32 command;

    // implement protection agains lost INT only in HUB and BACPLANE systems
    /* if((xpci_systemType==HUB)||(xpci_systemType==BACKPLANE)){
        while (it_cnt == 0){
            // 10000 value adjusted with the processor speed and the system
            // speed to answer with 8 modules maximums
            // if (delay < 5000){ //initial value by arek

            // test with 10000 and 1000000 give the same delay to read image but the larger gives less timeout errors
            // when the system is loaded. Simply in case of error it is detected faster and the process can go on
            // sooner avoid long gap in image taking.
            if (delay < 1000000){ // 300ms for 1000000
                delay++;
                sched_yield(); // give control to the scheduler
                //usleep(1);
            }
            else {
                // if we go here the IT has been probably lost or the
                // IT handler has not been executed. This handler resets
                // the IT register status. So we clear the IT register if
                // necessary
                printf("ERROR waitIT(): IT probably lost ... may need to adjust protection delay [xpci_it_pos =%d]\n",xpci_it_pos);
                command=0xffffffff;
                if (xpci_getInterruptStatus()==1)
                    PldaMemoryWrite32 (nIndex,BAR_0, 15, 1 , &command);
                error = -1;
                break;
            }
        }
    }
    // in case of any of the IMXPAD systems the protection against INT losts is not necessary
    else{
        while(it_cnt == 0);
    }
   */ // Notice: another way to wait for the IT occurence would have been to use
    // polling on the IT status
    // while (xpci_getInterruptStatus()==1);
    while(xpci_getItCount() == 0);
    
    initIt(); //prepare for next dma
    return error;
}

/***************************************************************
* Function to wait on the IT with a coarse software timeout 
* more or less in millisec
* timeout : software timeout
* 
* returns 0 succes on sync
*         -1 timeout
*
****************************************************************/
static int timedWaitIT( int timeout ){ 
    return timedWaitOnCond( timeout , 3, xpci_getItCnt);
}
/*
  int           delay = 0;

  int           ret, error;
  UINT32 command;

  xpci_timerStart(0);
  error = 0;
  // This value in usleep() is not very significant in detail because
  // a value 1 gives a delay from 100 to 200 one a value of 1000 for
  // timeout with an instable measure. Only after pushing the value at
  // 50 we can detect a real change of about 40% on average.
  while (it_cnt == 0){
    if (delay < timeout){
      delay++;
      ret = usleep((unsigned long)1);

    }
    else {
      // abort the pending transactions
      xpci_resetChannels(3); // 3 for all channels
      command=0xffffffff;
      if (xpci_getInterruptStatus()==1)
    PldaMemoryWrite32 (nIndex,BAR_0, 15, 1 , &command);
      xpci_timerStop(0); // prints the delay
      error = -1;
      break;
    }
  }
  //xpci_timerStop(0); // prints the delay
  return error;

  }*/

// This function is used to suspend the process in case of long exposition 
// to prevent an hardware timeout on the read. It is not necessary for
// fast acquisition but is necessary for more than 0,5sec delay.
//=========================================================================
static void xpci_delayWaitReply(int gateLength, int timeUnit){
    // Add by fred le 2/2/2011
    // wait until end of expose detector to prevent hardware read timeout to trigg
    int time;

    switch (timeUnit)
    {
    case 1 : // usec
        // useless in this case usleep(gateLength);
        break;
    case 2 : // millisec
        if (gateLength > 500) // only delay in this case FRED
            for(time=0;time<gateLength;time++)
                usleep(1);
        break;
    case 3: // sec
        sleep(gateLength);
        usleep(gateLength);
        break;
    default:
        printf("ERROR timeUnit = %d\n default Time = 1s\n\n",timeUnit);
        sleep(1);
    }
}
// Function to wait for data to receive on a specified channel
// This function returns an error if the softimeout is reached
// or the hardware timeout interrupts the transaction.
// INPUT: channel (which channel to check)
//        softTimeout (delay in usec for the timeout)
// RETURNS: 0 success
//          -1 error (timeout or hard error)
//============================================================
static int xpci_waitOnChannel(int channel, int softTimeout){
    int dmaStatus =0;
    int error;

    //  if (softTimeout==0)
    WAIT_IT(dmaStatus,softTimeout); // fast wait with short delay
    /*   dmaStatus = waitIT();
  else
  dmaStatus = timedWaitIT(softTimeout);*/

    if (dmaStatus == -1){
        printf("!!!! >>>>> !!!!! Soft timeout %s(%d) on DMA read it_cnt=%d\n",  __func__, channel, it_cnt);
    }
    // now check if we have an hardware error
    error = *(error_msg_channel[channel]) & 0xff;
    if (error){
        printf("ERROR: %s(%d) last DMA[%d] command svc status is 0x%x\n",  __func__, channel, channel, error);
        dmaStatus = -1;
    }
    return dmaStatus;
}

static int xpci_allocateDmaMemory(unsigned int *lockNb, SBufferDescription *bufDesc, int size){
    memset(bufDesc, 0, sizeof(SBufferDescription));
    bufDesc->ByteCount = SERV_BUFFER_SIZE;
    //printf("Alloc Phy on %d\n", lockNb);
    if ( PldaLockPhysicalAddress(nIndex, lockNb , bufDesc) == FALSE ){
        printf("ERROR: PldaLockPhysicalAddress() fails on %d\n", lockNb);
        return -1;
    }
    else
        return 0;
}
// Function to set to 0 the svc fifo dma target memory
//=====================================================
static void xpci_eraseSvcMsg(int channel){
    int chn;
    if (channel >= NB_CHANNELS){
        for(chn=0; chn < NB_CHANNELS; chn++){
            memset(error_msg_channel[chn], 0x00, SERV_BUFFER_SIZE);
        }
    }
    else
        if (channel==0)
            memset (error_msg_channel[0], 0x00, SERV_BUFFER_SIZE);
        else
            memset (error_msg_channel[1], 0x00, SERV_BUFFER_SIZE);
}
// Hard cleaning: 
// Resets all the hardware registers and reprograms the service DMA addresses registers
// This function can only be used once the xpci_init() has been done with success and the
// memory for service FIFOs has been allocated.
//===========================================================================================
int xpci_resetBoard(int det){
    unsigned long physAdd;

    if (detStatus[det] != UP){
        printf("ERROR: PCIe board should have been init before resetting it\n");
        return -1;
    }
    xpci_resetHardware(det);

    /* register write addresses for sevices error/status dma info */
    physAdd = svcDma0.PhysicalAddr & 0xFFFFFFFC;
    PldaMemoryWrite32 (nIndex, BAR_0, DMA0_SRV_PHYADD_OFFSET, 1, &physAdd);
    physAdd = svcDma1.PhysicalAddr & 0xFFFFFFFC;
    PldaMemoryWrite32 (nIndex, BAR_0, DMA1_SRV_PHYADD_OFFSET, 1,& physAdd);

    /* put the hard time out to the default value */
    xpci_setHardTimeout(TIMEOUT_HARD);
    usleep(500); // reset board takes time ....
    return 0;
}


/******************************************************
 * Init all channels even if only a few are used
* Set common IT function
* Create globals for driver sync and data
* Create memory for error messages
* det: the identification of the detector/plda board
*      only the number 0 is in use now
*******************************************************/
int xpci_init(int det, int sysType)
{
    int           result = 0, i;
    /*
  if (sysType != PLDA_PROTO_1){
    printf("%s() ERROR: Unknown system type %d\n",__func__,sysType);
    return -1;
    }*/
    switch(sysType){
    case HUB:
        xpci_systemChip = XPAD_31;
        xpci_systemType = sysType;
        break;
    case BACKPLANE:
        xpci_systemChip = XPAD_32;
        xpci_systemType = sysType;
        break;
    case IMXPAD_S70:
    case IMXPAD_S140:
    case IMXPAD_S340:
    case IMXPAD_S420:
    case IMXPAD_S540:
    case IMXPAD_S1400:
    case IMXPAD_S700:
        xpci_systemChip = XPAD_32;
        xpci_systemType = sysType;
        break;
    default:
        printf("ERROR: UNKNOWN detector type\n");
        return -1;
    }

    if(detStatus[det] == UP) {
        printf("INFO: det already UP\n");
        return 0;
    }

    /* Open driver for the PLDA board */
    if ( (result = PldaInitDriverFilter(PLDA_VENDOR_ID, 0)) > 0 ){
        cprintf (".. Failed to open driver ", RED);
        printf ("(err=0x%x) ", result);
        cprintf ("\n. Exit\n", RED);
        return -1; /* exit */
    }

    PldaGetPCIAgentID(0, &PCIslotDescription); // Get board information
    if( PCIslotDescription.VendorID == PLDA_VENDOR_ID){
        nIndex = det; // first is 0 for one plda board
        if ( PldaLockResources(nIndex) == FALSE ){
            printf("ERROR: PldaLockRessources() fails on %d\n", nIndex);
            return -1;
        }
    }
    else{
        cprintf(" No PLDA reference design found!\n. Exit.\n", BLUE);
        return -1;
    }

    /* Set common IT function */
    if ( PldaInitializeInterrupt(  nIndex,
                                   (SPCIInterruptTransfer* )&RefDesignInterruptInit,
                                   (INT_HANDLER_FUNC) &interruptHandler) != TRUE ){
        cprintf("Initialize interrupt failed\n",  RED);
        return -1;
    }

    /* allocate the 2 channels error/status memory containers */
    if ( xpci_allocateDmaMemory(&DMA0_SVC_BUF_LOCK, &svcDma0, SERV_BUFFER_SIZE)!=0){
        return -1;
    }

    error_msg_channel[0] = (char*)svcDma0.UserAddr;
    if ( xpci_allocateDmaMemory(&DMA1_SVC_BUF_LOCK, &svcDma1, SERV_BUFFER_SIZE)!=0) {
        //printf("Dealloc Phys on %d\n", DMA0_SVC_BUF_LOCK);
        PldaReleasePhysicalAddress(nIndex, DMA0_SVC_BUF_LOCK);
        return -1;
    }
    error_msg_channel[1] = (char*)svcDma1.UserAddr;
    detStatus[det] = UP;// should be set here to deallocate svc mem when xpci_close() is called

    xpci_resetBoard(det);    // checks that the board status is UP, reset board and init service FIFOs DMA memory  and hardware timeout
    xpci_setHardTimeout(HWTIMEOUT_1SEC);
    xpci_getHardTimeout(1);  //just printfs the current value in clear

    printf("%s(%d) hardware timeout has been initialized\n", __func__, HWTIMEOUT_1SEC);
    printf("%s() Current Status registers are:\n",__func__);
    xpci_dumpStatusRegsTable();
    flag_startExpose = 0;
    printf("\n");
    return 0;
}






void xpadModuleType( int *modules_nb , int *start_module){
    // different imxpad systems consist in different number of modules
    switch(xpci_systemType){
    case IMXPAD_S70:
        *modules_nb = 1;
        *start_module = 0;
        break;

    case IMXPAD_S140:
        *modules_nb = 2;
        *start_module = 0;
        break;
    case IMXPAD_S340:
        *modules_nb = 5;
        *start_module = 0;
        break;
    case IMXPAD_S420:
        *modules_nb = 6;
        *start_module = 2;
        break;
    case IMXPAD_S540:
    case HUB:
        *modules_nb = 8;
        *start_module = 0;
        break;
    case IMXPAD_S1400:
        *modules_nb = 20; //____
        *start_module = 0;
        break;
        case IMXPAD_S700:
        *modules_nb = 10; //____
        *start_module = 0;
        break;
    default:
        *modules_nb = 8;
        *start_module = 0;
        break;
    }
}



void xpci_close(int det){
    if (detStatus[det] != UP)
    {
        printf("detStatus[%d] != UP\n",det);
        return;
    }
    //printf("Clearing IT ...\n");
    PldaReleaseInterrupt(nIndex);
    //printf("Releases svc memory ...\n");
    printf("Dealloc Phys on %d\n", DMA0_SVC_BUF_LOCK);
    PldaReleasePhysicalAddress(nIndex, DMA0_SVC_BUF_LOCK);
    printf("Dealloc Phys on %d\n", DMA1_SVC_BUF_LOCK);
    PldaReleasePhysicalAddress(nIndex, DMA1_SVC_BUF_LOCK);
    /* relesae boerd resources */
    //printf("Clearing resources ...\n");
    PldaClearResources(nIndex);
    /* close the driver */
    //printf("Clearing Driver ...\n");
    PldaClearDriver();
}
/*********************************************************************
 * Function to round to a X bytes boundary a memory size given in bytes
**********************************************************************/
static int roundSizeToXBytes(int size, int nbBytes){
    int extbytes, msglen;
    extbytes = size%nbBytes;
    msglen   = size;
    if (extbytes){
        for (int i=0; i<(nbBytes-extbytes); i++){ msglen++; }
    }
    //printf("Real size for %d is %d\n", size, msglen);
    return msglen;
}
/**************************************************************
 * Function to diffuse the same message on the two channels
 * This function receives the number of bytes to write.
 * Current out TX buffer size is 2K. this function only
 * accepts a block of data up to 2K.
 *
 * If more data have to be sent use xpci_write() with
 * channel >=2 to send on both channels
 *
 * size : the size in bytes of the data to send. They are
 * strored in an array of 16 bits words.
 ***************************************************************/
int xpci_writeCommon(uint16_t *data, int size){
    SBufferDescription	tx0_buffer;
    SBufferDescription 	tx1_buffer;
    DWORD			dma_conf[10];
    uint16_t      	*ptx0, *ptx1;
    int                   j;
    UINT32         command;
    int                   dmaStatus;

    xpci_resetChannels(3); // reset the 2 channels
    xpci_eraseSvcMsg(3);// applied to both channels
    // ATTENTION: this init is necessary otherwise the allocation fails
    // memset (&tx0_buffer, 0, sizeof(SBufferDescription) );
    // memset (&tx1_buffer, 0, sizeof(SBufferDescription) );

    // Allocation  to send commands

    tx0_buffer.ByteCount = size;
    if ( !PldaLockPhysicalAddress(nIndex, &DMA0_TX_BUF_LOCK, &tx0_buffer) )
    {
        printf("..%s() failed to allocate memory (TX0 buffer).Exit\n", __func__);
        exit(0);
    }
    tx1_buffer.ByteCount = size;
    if ( !PldaLockPhysicalAddress(nIndex, &DMA1_TX_BUF_LOCK, &tx1_buffer))
    {
        PldaReleasePhysicalAddress(nIndex, DMA0_TX_BUF_LOCK);
        printf("..%s failed to allocate memory (TX1 buffer).Exit\n", __func__);
        exit(0);
    }
    dma_conf[0] = tx0_buffer.PhysicalAddr & 0xFFFFFFFC;
    dma_conf[1] = tx1_buffer.PhysicalAddr & 0xFFFFFFFC;
    dma_conf[2] = size;
    dma_conf[3] = size;
    PldaMemoryWrite32 (nIndex, BAR_0, 0, 4, dma_conf);
    
    ptx0 = (uint16_t*) tx0_buffer.UserAddr;
    ptx1 = (uint16_t*) tx1_buffer.UserAddr;
    if (debugMsg) printf("DEBUG: %s() sending on the 2 channels\n", __func__);

    for(j = 0; j<size/sizeof(uint16_t); j++)
    {
        *ptx0 = data[j]; if (debugMsg) printf(" 0x%04x ",*ptx0);
        *ptx1 = data[j];
        ptx0++;
        ptx1++;
    }

    if (debugMsg) printf("\n");
    ptx0 = (uint16_t*) tx0_buffer.UserAddr;
    ptx1 = (uint16_t*) tx1_buffer.UserAddr;

    command=START_TXDMA;
    //=========  send on the first channel
    initIt();//it_cnt = 0;
    PldaMemoryWrite32 (nIndex, BAR_0, 10, 1 , &command);
    dmaStatus = waitIT();//make IT loss !! dmaStatus = xpci_waitOnChannel(0,0);
    if (dmaStatus==-1){
        printf("ERROR in %s() on channel 0\n", __func__);
        goto endError;
    }

    //=========  send on the second channel
    initIt();//it_cnt=0;
    PldaMemoryWrite32 (nIndex, BAR_0, 11, 1 , &command);
    dmaStatus = waitIT();//make IT loss !!dmaStatus = xpci_waitOnChannel(0,0);
    if (dmaStatus==-1){
        printf("ERROR in %s on channel 1\n", __func__);
        goto endError;
    }
    PldaReleasePhysicalAddress(nIndex, DMA0_TX_BUF_LOCK);
    PldaReleasePhysicalAddress(nIndex, DMA1_TX_BUF_LOCK);
    return 0;
endError:
    PldaReleasePhysicalAddress(nIndex, DMA0_TX_BUF_LOCK);
    PldaReleasePhysicalAddress(nIndex, DMA1_TX_BUF_LOCK);
    return -1;
}




/**************************************************************
 * Function to diffuse the same message on the two channels
 * Add mask parameter beaucause the data transfer are 16 bits and the S1400 detector used 20 bits
 * This function receives the number of bytes to write.
 * Current out TX buffer size is 2K. this function only
 * accepts a block of data up to 2K.
 *
 * If more data have to be sent use xpci_write() with

 * channel >=2 to send on both channels
 *
 * size : the size in bytes of the data to send. They are
 * strored in an array of 16 bits words.
 ***************************************************************/
int xpci_writeCommon_S1400(uint16_t *data, int size,unsigned modMask){
    SBufferDescription	tx0_buffer;
    SBufferDescription 	tx1_buffer;
    DWORD			dma_conf[10];
    uint16_t      	*ptx0, *ptx1;
    int                   j;
    UINT32         command;
    int                   dmaStatus;

    xpci_resetChannels(3); // reset the 2 channels
    xpci_eraseSvcMsg(3);// applied to both channels
    // ATTENTION: this init is necessary otherwise the allocation fails
    memset (&tx0_buffer, 0, sizeof(SBufferDescription) );
    memset (&tx1_buffer, 0, sizeof(SBufferDescription) );

    // Allocation  to send commands
    tx0_buffer.ByteCount = size;
    if ( PldaLockPhysicalAddress(nIndex, &DMA0_TX_BUF_LOCK, &tx0_buffer) == FALSE )
    {
        printf("..%s() failed to allocate memory (TX0 buffer).Exit\n", __func__);
        exit(0);
    }
    tx1_buffer.ByteCount = size;
    if ( PldaLockPhysicalAddress(nIndex, &DMA1_TX_BUF_LOCK, &tx1_buffer) == FALSE )
    {
        PldaReleasePhysicalAddress(nIndex, DMA0_TX_BUF_LOCK);
        printf("..%s failed to allocate memory (TX0 buffer).Exit\n", __func__);
        exit(0);;
    }
    dma_conf[0] = tx0_buffer.PhysicalAddr & 0xFFFFFFFC;
    dma_conf[1] = tx1_buffer.PhysicalAddr & 0xFFFFFFFC;
    dma_conf[2] = size;
    dma_conf[3] = size;
    PldaMemoryWrite32 (nIndex, BAR_0, 0, 4, dma_conf);
    
    ptx0 = (uint16_t*) tx0_buffer.UserAddr;
    ptx1 = (uint16_t*) tx1_buffer.UserAddr;
    if (debugMsg) printf("DEBUG: %s() sending on the 2 channels\n", __func__);
    if(xpci_systemType==IMXPAD_S1400)
    {
        for(j = 0; j<size/sizeof(uint16_t); j++)
        {
            if((data[1] == MOD_MESSAGE) && (j == 3)){
                *ptx1 = (modMask & 0x03FF); if (debugMsg) printf("0x%04x ",*ptx1);
                *ptx0 =(((modMask >> 10) & 0x3FF) | 0x400);
                ptx0++;
                ptx1++;

            }
            else{
                *ptx0 = data[j]; if (debugMsg) printf("  0x%04x ",*ptx0);
                *ptx1 = data[j];
                ptx0++;
                ptx1++;
            }
        }
    }
    else
    {
        for(j = 0; j<size/sizeof(uint16_t); j++)
        {
            *ptx0 = data[j]; if (debugMsg) printf(" 0x%04x ",*ptx0);
            *ptx1 = data[j]; if (debugMsg) printf(" 0x%04x ",*ptx1);
            ptx0++;
            ptx1++;
        }
    }
    if (debugMsg) printf("\n");
    ptx0 = (uint16_t*) tx0_buffer.UserAddr;
    ptx1 = (uint16_t*) tx1_buffer.UserAddr;

    command=START_TXDMA;
    //=========  send on the first channel
    initIt();//it_cnt = 0;
    PldaMemoryWrite32 (nIndex,BAR_0, 10, 1 , &command);
    dmaStatus = waitIT();//make IT loss !! dmaStatus = xpci_waitOnChannel(0,0);
    if (dmaStatus==-1){
        printf("ERROR in %s() on channel 0\n", __func__);
        goto endError;
    }
    //=========  send on the second channel
    initIt();//it_cnt=0;
    PldaMemoryWrite32 (nIndex,BAR_0, 11, 1 , &command);
    dmaStatus = waitIT();//make IT loss !!dmaStatus = xpci_waitOnChannel(0,0);
    if (dmaStatus==-1){
        printf("ERROR in %s on channel 1\n", __func__);
        goto endError;
    }
    PldaReleasePhysicalAddress(nIndex, DMA0_TX_BUF_LOCK);
    PldaReleasePhysicalAddress(nIndex, DMA1_TX_BUF_LOCK);
    return 0;
endError:
    PldaReleasePhysicalAddress(nIndex, DMA0_TX_BUF_LOCK);
    PldaReleasePhysicalAddress(nIndex, DMA1_TX_BUF_LOCK);
    return -1;
}











/**************************************************************
 * This is a special version of the writeCommon() function for
 * speed optimization in readNextImage().
 * It uses preallocated transmit buffers (done in readImageInit()
 * to send image request messages.
 *
 * Function to diffuse the same message on the two channels
 * This function receives the number of bytes to write.
 * Current out TX buffer size is 2K. this function only
 * accepts a block of data up to 2K.
 *
 * If more data have to be sent use xpci_write() with
 * channel >=2 to send on both channels
 *
 * size : the size in bytes of the data to send. They are
 * strored in an array of 16 bits words.
 ***************************************************************/
int xpci_writeCommonNextImage(uint16_t *data, int size){
    DWORD			dma_conf[10];
    uint16_t      	*ptx0, *ptx1;
    int                   j;
    UINT32         command;
    int                   dmaStatus;

    xpci_resetChannels(3); // reset the 2 channels
    xpci_eraseSvcMsg(3);// applied to both channels

    dma_conf[0] = img_txBuffer0.PhysicalAddr & 0xFFFFFFFC;
    dma_conf[1] = img_txBuffer1.PhysicalAddr & 0xFFFFFFFC;
    dma_conf[2] = size;
    dma_conf[3] = size;
    PldaMemoryWrite32 (nIndex, BAR_0, 0, 4, dma_conf);

    ptx0 = (uint16_t*) img_txBuffer0.UserAddr;
    ptx1 = (uint16_t*) img_txBuffer1.UserAddr;
    if (debugMsg) printf("DEBUG: %s() sending on the 2 channels\n", __func__);
    for(j = 0; j<size/sizeof(uint16_t); j++)
    {
        *ptx0 = data[j]; if (debugMsg) printf("0x%04x ",*ptx0);
        *ptx1 = data[j];
        ptx0++;
        ptx1++;
    }
    if (debugMsg) printf("\n");
    ptx0 = (uint16_t*) img_txBuffer0.UserAddr;
    ptx1 = (uint16_t*) img_txBuffer1.UserAddr;

    command=START_TXDMA;
    //=========  send on the first channel
    initIt();//it_cnt = 0;
    PldaMemoryWrite32 (nIndex,BAR_0, 10, 1 , &command);
    dmaStatus = waitIT();//make IT loss !! dmaStatus = xpci_waitOnChannel(0,0);
    if (dmaStatus==-1){
        printf("ERROR in %s() on channel 0\n", __func__);
        return -1;
    }
    //=========  send on the second channel
    initIt();//it_cnt=0;
    PldaMemoryWrite32 (nIndex,BAR_0, 11, 1 , &command);
    dmaStatus = waitIT();//make IT loss !!dmaStatus = xpci_waitOnChannel(0,0);
    if (dmaStatus==-1){
        printf("ERROR in %s on channel 1\n", __func__);
        return -1;
    }
    return 0;
}
/**************************************************************
 * size : the size in bytes of the data to send. They are
 *        passed in an array of 16 bits.
 * noReset: flag to say that no reset should be made on the PCI
 *          borad. Usefull for PCI loop mode test not to erase
 *          the loop mode information.
 ***************************************************************/
static int xpci_writeSplitted(int channel, uint16_t *data, int size){
    int                     i, chn, curchn;
    int                     inUse[2];   // use to know which channel to use
    int                     cmdAlloc[2]; // used to know which channels have been allocated
    unsigned int long       addOffset[] ={DMA0_TX_PHYADD_OFFSET, DMA1_TX_PHYADD_OFFSET};
    unsigned int long       sizeOffset[]={DMA0_TX_SIZE_OFFSET, DMA1_TX_SIZE_OFFSET};
    unsigned int long       cmdOffset[] ={DMA0_CTRL,DMA1_CTRL};
    int                     lockNb[]={DMA0_TX_BUF_LOCK,DMA1_TX_BUF_LOCK};
    SBufferDescription 	  wrBuffer[NB_CHANNELS];
    uint16_t                 *pval;
    int                     dmaStatus[NB_CHANNELS];
    int                     error;
    unsigned long          dmaAddr[NB_CHANNELS];
    unsigned long           msglen;
    unsigned long           command = START_TXDMA;

    error = 0;

    for (i=0; i< NB_CHANNELS; i++) { inUse[i]=0; cmdAlloc[i]=0;}
    if (channel >= NB_CHANNELS) {inUse[0]=1; // both channels are used
        inUse[1]=1;}
    else                     {inUse[channel]=1;}
    /*for (i=0; i< NB_CHANNELS; i++)
    printf("xpci_writeSplitted(): Channel %d use status is %d\n", i, inUse[i]);*/
    xpci_eraseSvcMsg(channel);
    //erase dma command parameters
    for (chn=0; chn<NB_CHANNELS; chn++)
        memset (&(wrBuffer[chn]), 0, sizeof(SBufferDescription) );

    // ATTENTION THIS IS REALY NECESSARY To PREVENT UNEXPLAINED HAZARD AND CUMULATIV ERRORS
    xpci_resetChannels(channel);

    msglen =  roundSizeToXBytes(size,8);
    /*
     Allocate contiguous system memory buffer and returns its physical address
    */
    for (chn=0; chn<NB_CHANNELS; chn++){
        if (inUse[chn]){
            // printf(".... writing ... channel %d is in use\n", chn);
            wrBuffer[chn].ByteCount = msglen;
            //  printf("Alloc Phy on %d\n", lockNb[chn]);
            if(chn == 0){
                if ( PldaLockPhysicalAddress(nIndex, &DMA0_TX_BUF_LOCK , &(wrBuffer[chn])) == FALSE ) {
                    printf("ERROR: PldaLockPhysicalAddress() fails on lock %d for channel %d\n", lockNb[chn], chn);
                    for (curchn=0; curchn<chn; curchn++){
                        if (inUse[curchn]){
                            if(curchn == 0)
                                PldaReleasePhysicalAddress(nIndex, DMA0_TX_BUF_LOCK);
                            else
                                PldaReleasePhysicalAddress(nIndex, DMA1_TX_BUF_LOCK);
                        }
                        return -1;
                    }
                }//lock error
                else
                    cmdAlloc[chn]=1;
            }
            else{
                if ( PldaLockPhysicalAddress(nIndex, &DMA1_TX_BUF_LOCK , &(wrBuffer[chn])) == FALSE ) {
                    printf("ERROR: PldaLockPhysicalAddress() fails on lock %d for channel %d\n", lockNb[chn], chn);
                    for (curchn=0; curchn<chn; curchn++){
                        if (inUse[curchn]){
                            if(curchn == 0)
                                PldaReleasePhysicalAddress(nIndex, DMA0_TX_BUF_LOCK);
                            else
                                PldaReleasePhysicalAddress(nIndex, DMA1_TX_BUF_LOCK);
                        }
                    }
                    return -1;
                }//lock error
                else
                    cmdAlloc[chn]=1;

            }

            pval =  (uint16_t*)(wrBuffer[chn].UserAddr);
            //Copy user data to the DMA source mem
            for(i = 0; i<size/sizeof(uint16_t); i++)
            {
                *pval = data[i]; // here only low order bits are copied the other are lost
                pval++;
            }
            if (debugMsg){
                pval =  (uint16_t*)(wrBuffer[chn].UserAddr);
                printf("DEBUG: xpci_sending on channel %d:", chn);
                for(i = 0; i<size/sizeof(uint16_t); i++)
                {
                    printf("0x%04x ",*pval);
                    pval++;
                }
                printf("\n");
            }
            /* send data */
            dmaAddr[chn] = wrBuffer[chn].PhysicalAddr & 0xFFFFFFFC;  /* Set low read physical address rounded on 16 bit word */

            PldaMemoryWrite32 (nIndex, BAR_0, addOffset[chn],  1, &(dmaAddr[chn]));
            PldaMemoryWrite32 (nIndex, BAR_0, sizeOffset[chn], 1, &msglen);
        }//inUse
    }// for chn

    // for timing reasons try to send both commands as close as possible one from another. This is the reason why the
    // parameter setting loop and this sending one are separated
    
    for (chn=0; chn<NB_CHANNELS; chn++){
        dmaStatus[chn] = 0;
        if (inUse[chn]){
            initIt();
            PldaMemoryWrite32 (nIndex, BAR_0, cmdOffset[chn], 1 , &command); /* start DMA */

            /* wait IT timeout in millisec */
            dmaStatus[chn] = waitIT( TIMEOUT_CNT);

            if (dmaStatus[chn] == -1){ // soft timeout
                printf("!!!! >>>>> !!!!! soft timeout on DMA write it_cnt=%d on channel %d\n", it_cnt, chn);
            }
            // get hardware error among which hardware timeout
            error = *(error_msg_channel[chn]) & 0xff;
            if (error){
                printf("ERROR: %s() last DMA[%d] command svc status is 0x%x\n",  __func__,chn, error);
                dmaStatus[chn] = -1;
            }
        }//inUse
    }//for chn

    for (chn=0; chn<NB_CHANNELS; chn++){
        if (cmdAlloc[chn]){
            //printf("Dealloc Phys on %d\n", lockNb[chn]);
            //  PldaReleasePhysicalAddress(nIndex, lockNb[chn]);
            if(chn == 0)
                PldaReleasePhysicalAddress(nIndex, DMA0_TX_BUF_LOCK);
            else
                PldaReleasePhysicalAddress(nIndex, DMA1_TX_BUF_LOCK);
        }
    }
    // computes and returns the final write() transfer status
    if ((dmaStatus[0]+dmaStatus[1])==0) //check they have all been initialized even if not used
        return 0;
    else
        return -1;
}



static int xpci_writeSplitted_resetTX(int channel, uint16_t *data, int size){
    int                     i, chn, curchn;
    int                     inUse[2];   // use to know which channel to use
    int                     cmdAlloc[2]; // used to know which channels have been allocated
    unsigned int long       addOffset[] ={DMA0_TX_PHYADD_OFFSET, DMA1_TX_PHYADD_OFFSET};
    unsigned int long       sizeOffset[]={DMA0_TX_SIZE_OFFSET, DMA1_TX_SIZE_OFFSET};
    unsigned int long       cmdOffset[] ={DMA0_CTRL,DMA1_CTRL};
    int                     lockNb[]={DMA0_TX_BUF_LOCK,DMA1_TX_BUF_LOCK};
    SBufferDescription 	  wrBuffer[NB_CHANNELS];
    uint16_t                 *pval;
    int                     dmaStatus[NB_CHANNELS];
    int                     error;
    unsigned long          dmaAddr[NB_CHANNELS];
    unsigned long           msglen;
    unsigned long           command = START_TXDMA;

    error = 0;

    for (i=0; i< NB_CHANNELS; i++) { inUse[i]=0; cmdAlloc[i]=0;}
    if (channel >= NB_CHANNELS) {inUse[0]=1; // both channels are used
        inUse[1]=1;}
    else                     {inUse[channel]=1;}
    /*for (i=0; i< NB_CHANNELS; i++)
    printf("xpci_writeSplitted(): Channel %d use status is %d\n", i, inUse[i]);*/
    xpci_eraseSvcMsg(channel);
    //erase dma command parameters
    for (chn=0; chn<NB_CHANNELS; chn++)
        memset (&(wrBuffer[chn]), 0, sizeof(SBufferDescription) );

    // ATTENTION THIS IS REALY NECESSARY To PREVENT UNEXPLAINED HAZARD AND CUMULATIV ERRORS
    xpci_resetTXChannels(channel);

    msglen =  roundSizeToXBytes(size,8);
    /*
     Allocate contiguous system memory buffer and returns its physical address
    */
    for (chn=0; chn<NB_CHANNELS; chn++){
        if (inUse[chn]){
            // printf(".... writing ... channel %d is in use\n", chn);
            wrBuffer[chn].ByteCount = msglen;
            //  printf("Alloc Phy on %d\n", lockNb[chn]);
            if(chn == 0){
                if ( PldaLockPhysicalAddress(nIndex, &DMA0_TX_BUF_LOCK , &(wrBuffer[chn])) == FALSE ) {
                    printf("ERROR: PldaLockPhysicalAddress() fails on lock %d for channel %d\n", lockNb[chn], chn);
                    for (curchn=0; curchn<chn; curchn++){
                        if (inUse[curchn]){
                            if(curchn == 0)
                                PldaReleasePhysicalAddress(nIndex, DMA0_TX_BUF_LOCK);
                            else
                                PldaReleasePhysicalAddress(nIndex, DMA1_TX_BUF_LOCK);
                        }
                        return -1;
                    }
                }//lock error
                else
                    cmdAlloc[chn]=1;
            }
            else{
                if ( PldaLockPhysicalAddress(nIndex, &DMA1_TX_BUF_LOCK , &(wrBuffer[chn])) == FALSE ) {
                    printf("ERROR: PldaLockPhysicalAddress() fails on lock %d for channel %d\n", lockNb[chn], chn);
                    for (curchn=0; curchn<chn; curchn++){
                        if (inUse[curchn]){
                            if(curchn == 0)
                                PldaReleasePhysicalAddress(nIndex, DMA0_TX_BUF_LOCK);
                            else
                                PldaReleasePhysicalAddress(nIndex, DMA1_TX_BUF_LOCK);
                        }
                    }
                    return -1;
                }//lock error
                else
                    cmdAlloc[chn]=1;

            }

            pval =  (uint16_t*)(wrBuffer[chn].UserAddr);
            //Copy user data to the DMA source mem
            for(i = 0; i<size/sizeof(uint16_t); i++)
            {
                *pval = data[i]; // here only low order bits are copied the other are lost
                pval++;
            }
            if (debugMsg){
                pval =  (uint16_t*)(wrBuffer[chn].UserAddr);
                printf("DEBUG: xpci_sending on channel %d:", chn);
                for(i = 0; i<size/sizeof(uint16_t); i++)
                {
                    printf("0x%04x ",*pval);
                    pval++;
                }
                printf("\n");
            }
            /* send data */
            dmaAddr[chn] = wrBuffer[chn].PhysicalAddr & 0xFFFFFFFC;  /* Set low read physical address rounded on 16 bit word */

            PldaMemoryWrite32 (nIndex, BAR_0, addOffset[chn],  1, &(dmaAddr[chn]));
            PldaMemoryWrite32 (nIndex, BAR_0, sizeOffset[chn], 1, &msglen);
        }//inUse
    }// for chn

    // for timing reasons try to send both commands as close as possible one from another. This is the reason why the
    // parameter setting loop and this sending one are separated
    
    for (chn=0; chn<NB_CHANNELS; chn++){
        dmaStatus[chn] = 0;
        if (inUse[chn]){
            initIt();
            PldaMemoryWrite32 (nIndex, BAR_0, cmdOffset[chn], 1 , &command); /* start DMA */

            /* wait IT timeout in millisec */
           // dmaStatus[chn] = waitIT( TIMEOUT_CNT);

            if (dmaStatus[chn] == -1){ // soft timeout
                printf("!!!! >>>>> !!!!! soft timeout on DMA write it_cnt=%d on channel %d\n", it_cnt, chn);
            }
            // get hardware error among which hardware timeout
            error = *(error_msg_channel[chn]) & 0xff;
            if (error){
                printf("ERROR: %s() last DMA[%d] command svc status is 0x%x\n",  __func__,chn, error);
                dmaStatus[chn] = -1;
            }
        }//inUse
    }//for chn

    for (chn=0; chn<NB_CHANNELS; chn++){
        if (cmdAlloc[chn]){
            //printf("Dealloc Phys on %d\n", lockNb[chn]);
            //  PldaReleasePhysicalAddress(nIndex, lockNb[chn]);
            if(chn == 0)
                PldaReleasePhysicalAddress(nIndex, DMA0_TX_BUF_LOCK);
            else
                PldaReleasePhysicalAddress(nIndex, DMA1_TX_BUF_LOCK);
        }
    }
    // computes and returns the final write() transfer status
    if ((dmaStatus[0]+dmaStatus[1])==0) //check they have all been initialized even if not used
        return 0;
    else
        return -1;
}



/****************************************************************
 * This function receives the number of bytes to write.
 * Current out TX buffer size is 2K so bigger data should
 * be splitted in several conscecutive transmissions.
 * channel <=2 means write on the two channels
 *
 * data   : passed data stored in an array of 16 bits words.
 * size   : the size in bytes of the data to send.
 * noReset: flag to say that no reset should be made on the PCI
 *          borad. Usefull for PCI loop mode test not to erase
 *          the loop mode information.
 ***************************************************************/
int xpci_write(int channel, uint16_t *data, int size){
    return xpci_writeExec(channel, data, size, 0);
}
int xpci_writeTestPCI(int channel, uint16_t *data, int size){
    return xpci_writeExec(channel, data, size, 1);
}
static int xpci_writeExec(int channel, uint16_t *data, int size, int noReset){
    int i, nbBlocks, lastBlockSize;
    int status = 0;
    int ret = 0;
    nbBlocks      = size / MAX_TX_FIFO_SIZE;
    lastBlockSize = size % MAX_TX_FIFO_SIZE;
    for (i=0; i<nbBlocks; i++){
        ret = xpci_writeSplitted( channel,
                                  (data+(i*MAX_TX_FIFO_SIZE)),
                                  MAX_TX_FIFO_SIZE);
        if (ret != 0)
            status = -1;
    }
    if (lastBlockSize != 0){
        ret = xpci_writeSplitted( channel,
                                  (data+(nbBlocks*MAX_TX_FIFO_SIZE)),
                                  lastBlockSize);
        if (ret != 0)
            status = -1;
    }
    return status;
}
/***********************624*******************************
* ATTENTION: performance issue
* Those functions are simple. It could be more efficient as it copies
* the recieved data in the buffer which address is passed by
* the caller.
* A future version should be developped to get back only the
* pointer to the received data but the user will have to
* deallocat the memory by itself.
* The user should pass an allocated space of size "size" or
* the function will crash.
*******************************************************/

/* Allocates the physical memory for transfer 
   Starts the transfer
   Waits transfer end
   Copys the received data in the passed buffer
   Frees the physical memory
   timeout = 0 use the hardware timeout
   timeout !=0 use the software timeout value is in ms
*******************************************************/ 
int xpci_read(int channel, uint16_t *data, int size, int timeout){ // dma transfer size in bytes
    unsigned int long     addOffset, sizeOffset, cmdOffset;

    SBufferDescription 	rdBuffer;
    int                   i, dmaStatus;
    unsigned            lockNb[2];
    uint16_t              *pval;
    UINT32         dmaAddr;
    UINT32         command = START_RXDMA;
    
    unsigned DMA_SELECT;

    if (size%8 != 0){
        printf("ERROR: %s() read size should be multiple of 8bytes\n", __func__);
        return -1;
    }
    switch(channel){
    case 0:
        addOffset = DMA0_RX_PHYADD_OFFSET;
        sizeOffset= DMA0_RX_SIZE_OFFSET;
        cmdOffset = DMA0_CTRL;
        break;
    case 1:
        addOffset = DMA1_RX_PHYADD_OFFSET;
        sizeOffset= DMA1_RX_SIZE_OFFSET;
        cmdOffset = DMA1_CTRL;
        break;
    default:
        cprintf("Unknown communication channel in xpci_read()\n",  RED);
        return -1;
    }
    dmaStatus = 0;
    xpci_eraseSvcMsg(channel);
    // NO usually when this function is called when data are already waiting in FIFOs xpci_resetChannels(3);
    //memset (&rdBuffer, 0, sizeof(SBufferDescription) );
    /*
    The message granularity on the FIFO internal bus is 64 bits OR 4 uint16_t OR 8 bytes.
    If too short can be padded with zeroes to round up
  */

    rdBuffer.ByteCount = size;
    /* Allocate contiguous memory
     allocates a system memory buffer and returns its physical address
    */
    // printf("Alloc Phy on lock %d\n", lockNb);
    // if(channel == 0){
    if ( PldaLockPhysicalAddress(nIndex, &DMA_SELECT , &rdBuffer) == FALSE ) {
        printf("ERROR: PldaLockPhysicalAddress() fails on %d\n", DMA0_RX_BUF_LOCK);
        return -1;
    }
    /*  }
    else{
        if ( PldaLockPhysicalAddress(nIndex, &DMA1_RX_BUF_LOCK , &rdBuffer) == FALSE ) {
            printf("ERROR: PldaLockPhysicalAddress() fails on %d\n", DMA1_RX_BUF_LOCK);
            return -1;
        }
    }
    */
    // memset ((void*)(rdBuffer.UserAddr), 0xde, size );// just scramble to be sure not to read old mem data from tx buff
    dmaAddr = rdBuffer.PhysicalAddr & 0xFFFFFFFC;  /* Set low read physical address */
    PldaMemoryWrite32 (nIndex, BAR_0, addOffset,  1, &dmaAddr);
    //PldaMemoryWrite32 (nIndex, BAR_0, sizeOffset, 1, &msglen);
    PldaMemoryWrite32 (nIndex, BAR_0, sizeOffset, 1, (unsigned long*)&size);

    // WAIT IN SOFT FOr DATA AVAILABLE
    // The read action is subject to hardware timeout limited to 8sec so for long exposition
    // we should use a software method.
    // So we wait that something is received in the RX fifo with a timeout before trying
    // to read with the DMA subject to the hardware timeout.
    // note: this work if the rx fifo has been resetted just before by the command sending
    //
    // xpci_timerStart(0);

    if (xpci_waitDataInFifo(channel, timeout)==-1){
        if (debugMsg)printf("DEBUG: %s(%d) timeout error\n",  __func__,channel);
        dmaStatus = -1; goto endFunc;
    }
    // now we know some data are available so we can start the DMA beeing sure that at least
    // some data are waiting in the fifo (if not all yet)
    initIt();
    PldaMemoryWrite32 (nIndex, BAR_0, cmdOffset, 1 , &command); /* start DMA */
    dmaStatus = xpci_waitOnChannel(channel, timeout);

    /* copy back the data for the user */
    /* The message granularity on the FIFO internal bus is 64 bits OR 4 uint16_t OR 8 bytes.
     So we know that we are aligned on a 32 bits boubdary */
    /* copy word by word but if a multiple of 4 bytes is not aligned on word
     add the necesary 0 */
    //msglen =  roundSizeToXBytes(size,4);
    //memset (data, 0, msglen ); /* will crash the program if the user as not allocated enough */
    memset (data, 0, size );/* will crash the program if the user as not allocated enough */
    pval =  (uint16_t*)(rdBuffer.UserAddr);
    for(int i = 0; i<size/(sizeof(uint16_t)); i++)
    {
        data[i] = pval[i];
    }
    if (debugMsg)
    {
        printf("DEBUG: %s(%d) received dump the target memory\n",  __func__,channel);
        for(i = 0; i<size/sizeof(uint16_t); i++)
        {
            printf("  0x%04x ",pval[i]);

        }
        printf("\n");
    }

endFunc:
    //if(channel == 0)
    PldaReleasePhysicalAddress(nIndex, DMA_SELECT);
    //	else
    //		PldaReleasePhysicalAddress(nIndex, DMA1_RX_BUF_LOCK);
    return dmaStatus;
}

void printUserBuffer(unsigned *userAddr, int nbLine){
    uint16_t	*ptx;
    int            j;
    int 		size;

	if( img_type == B2 )
		size = 566;
	else
		size = 1126;

    if (userAddr == 0) // just a hack for debugging xpci_arekReadImage()
        userAddr = (unsigned *)spy_mem_add;

    ptx = (uint16_t *)userAddr;
    for(j=0; j<(nbLine*size); j++ )
    {
        if (j%size==0){
            printf("\n");
        }
        if (j%size<8){
            printf("0x%x ",(unsigned)(*ptx));
        }
        if(j%size == size -1)
			printf("\t0x%x ",(unsigned)(*ptx));
			
        ptx++;
    }
    printf("\n");
}

/*************************************************************
   Function to start a DMA read on one channel.
   Transfer parameters should have already been set before.
**************************************************************/
void startDMARead(int channel){
    DWORD                 command       =  START_RXDMA;
    //initIt();
    xpci_eraseSvcMsg(channel);
    if (channel==0)
        PldaMemoryWrite32 (nIndex,BAR_0, 10, 1 , &command);
    else
        PldaMemoryWrite32 (nIndex,BAR_0, 11, 1 , &command);
}

/* Commodity function to compute the image size and image transfer parameters out
   of the number of used chips on the module
   IN  : nbchips
   OUT : lineSize             length of an image line
         img_tranferSize      size of data image transfer unit
         img_sizeImage        complete image size
*********************************************************************************/
void xpci_getImgDataParameters(enum IMG_TYPE type, int nbChips, int *lineSize, 
                               int *img_tranferSize, int *img_sizeImage){

    int nb_transfer; // nb transfers per image

    if (type==B4){
        nb_transfer = 3;
        *lineSize           = (nbChips * NB_COLUMN_CHIP)*2+6; // in words
        // one third a module image in bytes is the transfer unit <128K
        *img_tranferSize    = 2 * ((NB_LINE_CHIP/nb_transfer) * (*lineSize));
        *img_sizeImage      = (*img_tranferSize) * nb_transfer; // in bytes
    }
    else { //if(type==B2) or else
        nb_transfer = 2;
        *lineSize           = (nbChips * NB_COLUMN_CHIP)+6; // in words
        // half a module image in bytes is the transfer unit <128K
        *img_tranferSize    = 2 * ((NB_LINE_CHIP/nb_transfer) * (*lineSize));
        // *img_tranferSize    = 1 * ((NB_LINE_CHIP/nb_transfer) * (*lineSize));
        *img_sizeImage      = (*img_tranferSize) * nb_transfer; // in bytes
    }
}

/* Commodity function to compute the total number of modules out
   of the module mask.
   IN  : modMask
   RETRUN : total number of connected modules
*************************************************************************/
int  xpci_getModNb(unsigned modMask){
    int i;
    int nbMod0=0;
    int nbMod1=0;

    if( xpci_systemType == IMXPAD_S1400 || xpci_systemType == IMXPAD_S700)
    {
        for (i=0; i<20; i++)
            if(modMask & (1<<i)) (nbMod1)++;
        return (nbMod1);
    }
    if( xpci_systemType == IMXPAD_S700)
    {
        for (i=0; i<10; i++)
            if(modMask & (1<<i)) (nbMod1)++;
        return (nbMod1);
    }

    // PYD channel 0 and 1 are inverted
    for (i=0; i<4; i++)
        if(modMask & (1<<i)) (nbMod1)++;
    for (i=4; i<8; i++)
        if(modMask & (1<<i)) (nbMod0)++;
    return (nbMod0+nbMod1);
}

int  xpci_getLastMod(unsigned modMask){
    int i;
    int LastMod=0;

    if( xpci_systemType == IMXPAD_S1400 || xpci_systemType == IMXPAD_S700)
    {
        for (i=0; i<20; i++)
            if(modMask & (1<<i)) LastMod = i+1;
    }
    else    if(xpci_systemType == IMXPAD_S700)
    {
        for (i=0; i<10; i++)
            if(modMask & (1<<i)) LastMod = i+1;
    }
    else
    {
        // PYD channel 0 and 1 are inverted
        for (i=0; i<8; i++)
            if(modMask & (1<<i)) LastMod = i+1;
    }

    return LastMod;
}




int  xpci_getFirstMod(unsigned modMask){
    int i;
    int nbMod0=0;
    int nbMod1=0;

    if( xpci_systemType == IMXPAD_S1400 || xpci_systemType == IMXPAD_S700)
    {
        for (i=0; i<10; i++)
            if(modMask & (1<<i)) return i;
        for (i=10; i<20; i++)
            if(modMask & (1<<i)) return i;
        return -1;
    }    
    else   if(xpci_systemType == IMXPAD_S700)
    {
        for (i=0; i<5; i++)
            if(modMask & (1<<i)) return i;
        for (i=5; i<10; i++)
            if(modMask & (1<<i)) return i;
        return -1;
    }

    // PYD channel 0 and 1 are inverted
    for (i=0; i<4; i++)
        if(modMask & (1<<i)) return i;
    for (i=4; i<8; i++)
        if(modMask & (1<<i)) return i;
    return -1;
}
/* Commodity function to compute the number of modules on one channel out
   of the module mask.
   IN  : modMask
         channel (the channel id)
   RETRN : number of modules connected to channel 'channel'
*************************************************************************/
int  xpci_getModNbOnChnl(unsigned modMask, int channel){
    int i;
    int nbMod=0;
    
    
    if( xpci_systemType == IMXPAD_S1400)
    {
        switch (channel){
        case (1):
            for (i=0; i<10; i++)
                if(modMask & (1<<i)) (nbMod)++;
            break;
        case(0):
            for (i=10; i<20; i++)
                if(modMask & (1<<i)) (nbMod)++;
            break;
        default:
            nbMod = 0;
        }
        return (nbMod);
    }
    else if( xpci_systemType == IMXPAD_S700)
    {
        switch (channel){
        case (1):
            for (i=0; i<5; i++)
                if(modMask & (1<<i)) (nbMod)++;
            break;
        case(0):
            for (i=5; i<10; i++)
                if(modMask & (1<<i)) (nbMod)++;
            break;
        default:
            nbMod = 0;
        }
        return (nbMod);
    }
    else{
		// PYD channel 0 and 1 are inverted
		switch (channel){
		case (1):
			for (i=0; i<4; i++)
				if(modMask & (1<<i)) (nbMod)++;
			break;
		case(0):
			for (i=4; i<8; i++)
				if(modMask & (1<<i)) (nbMod)++;
			break;
		default:
			nbMod = 0;
		}
		return (nbMod);
	}
}
/**************************************************************************************
                            FAST IMAGE AQUISITION SEQUENCE

   Functions to read image or flat config which produce the same data volume and
   outputs from modules.

   Those 3 function are designed to optimize speed in burst acquisition of images.

   The first function to call is:
   int xpci_readImageInit() to allocate ressources and compute parameters stored in globals

   Then for the N images to acquire call N time:
   int xpci_readNextImage()

   To close the image acquisition sequence and relesae the ressources call:
   int xpci_readImageClose()
****************************************************************************************/
/***************************************************************************************
   Function to compute ressources necessary for the image acquisition and allocate them
   in globals variables.

   INPUT: type             type of image 2B or 4B bytes
          modulesMask      modules to get images from
          nbChips          number of chips to read per module
          data             address of the user buffer to fill with the data (should be allocated)

   The maximum FIFO size is 128K. So if we should transfer more we should do several
   transfer. To be faster we use the two channels in parallel one after the other copying
   the data received from one in the user data area while the other DMA is running. So the
   best average speed is obtained if the number of modules is equal for the two channels.

   ANALYSE 2B image:
   For a module a line length is
   nbChips * nbColum = nbWords with 7 chips and 80 col = 560 words (16bits words))
   But we should add 6 words per line in the message for the protocol:
   (nbChips * nbColum)+6 = 566 words
   ((nbChips * nbColum)+6)*2 = 1132 bytes per line
   For the full image : the 120 lines of a module this gives
   120*1132 = 135840 Bytes per module
   This higher than the maximum 128K FIFO size. We will thus split in half image transfers of
   60*566*2 = 67920 (0x10950) blocs by DMA and we need 2 transfers per module

   We will use 2 transfers by image in all cases (even if less chips are used)

   ANALYSE 4B image:
   For a module a line length is
   nbChips * nbColum = nbDWords with 7 chips and 80 col = 560 dwords (32bits words))
   But we should add 6 words per line in the message for the protocol:
   (nbChips * nbColum)*2+6 = 1126 words
   ((nbChips * nbColum)*2+6)*2 = 2252 bytes per line
   For the full image : the 120 lines of a module this gives
   120*2252 = 270240 Bytes per module
   This higher than the maximum 128K FIFO size. We will thus split in three image transfers of
   40*2252 = 90080 (0x15FE0) blocs by DMA and we need 3 transfers per module

   We will use 3 transfers by image in all cases (even if less chips are used)

   The data will be copied in the passed data buffer in sequence in increasing order
   of the module id: 1, 2 etc ... 8 (for the present modules)
****************************************************************************************/ 
int xpci_readImageInit(enum IMG_TYPE type, int moduleMask, int nbChips){ // dma transfer size in bytes
    UINT32         dmaAddr;
    int                   totalSizeToTransfer;
    int                   lineSize;
    // For 1 module we optimize with 2 transfers because the RX FIFO is 128K and 1/2 module image is a
    // maximum of 67920 bytes.
    // We adopt this value as the transfer unit whatever the number of modules connected is
    int                   transPerImage;

    img_type = type;

    if(img_type==B2) {
        transPerImage  = 2;
    }
    else if (img_type==B4) {
        transPerImage  = 3;
    }
    else {
        printf("ERROR: unknown image type\n");
        return -1;
    }

    // store the configuration parameter for later use
    img_moduleMask = moduleMask;

    // make a hard clean
    xpci_resetBoard(0);
    if(xpci_systemType==IMXPAD_S1400 || xpci_systemType==IMXPAD_S700){
        xpci_regRstFifo();
    }
    // compute the data volumes per module
    xpci_getImgDataParameters(img_type, nbChips, &lineSize, &img_transferSize, &img_sizeImage);
    if (debugMsg)
        printf("Image size is %d\n", img_sizeImage);

    // compute the number of modules/tranfers on each side
    // xpci_getModNb(moduleMask, &img_nbMod0, &img_nbMod1);
    img_nbMod0 = xpci_getModNbOnChnl(moduleMask, 0);
    img_nbMod1 = xpci_getModNbOnChnl(moduleMask, 1);

    if (debugMsg) printf("%d boards on channel 0 %d boards on channel 1\n",img_nbMod0,img_nbMod1);
    totalSizeToTransfer = (img_nbMod0+img_nbMod1)*img_transferSize*transPerImage; // need transPerImage transfer per connected modules

    if (img_nbMod1<img_nbMod0){
        img_nbParallelTrans=img_nbMod1*transPerImage;
        img_nbSeqTrans=(img_nbMod0-img_nbMod1)*transPerImage;
    }
    else {
        img_nbParallelTrans=img_nbMod0*transPerImage;
        img_nbSeqTrans=(img_nbMod1-img_nbMod0)*transPerImage;
    }
    if (debugMsg){
        printf("%d parallel transfers\n", img_nbParallelTrans);
        printf("%d sequential transfers\n", img_nbSeqTrans);
        printf("totalSizeToTransfer=%d\n", totalSizeToTransfer);
    }

    // ================ set para for transfer 0 ===============
    memset (&img_rdBuffer0, 0, sizeof(SBufferDescription) );
    // Allocate contiguous system memory buffer and returns its physical address
    img_rdBuffer0.ByteCount = img_transferSize;
    if ( PldaLockPhysicalAddress(nIndex, &DMA0_RX_BUF_LOCK , &img_rdBuffer0) == FALSE ) {
        printf("ERROR: PldaLockPhysicalAddress() fails on %d\n", DMA0_RX_BUF_LOCK);
        return -1;
    }
    dmaAddr = img_rdBuffer0.PhysicalAddr & 0xFFFFFFFC;  /* Set low read physical address */
    // Program DMA registers

    PldaMemoryWrite32 (nIndex, BAR_0, DMA0_RX_PHYADD_OFFSET, 1, &dmaAddr);
    PldaMemoryWrite32 (nIndex, BAR_0, DMA0_RX_SIZE_OFFSET,   1, (unsigned long*)&img_transferSize);

    // ============= set para for transfer 1 ===============
    memset (&img_rdBuffer1, 0, sizeof(SBufferDescription) );
    img_rdBuffer1.ByteCount = img_transferSize;
    // Allocate contiguous system memory buffer and returns its physical address
    if ( PldaLockPhysicalAddress(nIndex, &DMA1_RX_BUF_LOCK , &img_rdBuffer1) == FALSE ) {
        printf("ERROR: PldaLockPhysicalAddress() fails on %d\n", DMA1_RX_BUF_LOCK);
        PldaReleasePhysicalAddress(nIndex, DMA1_RX_BUF_LOCK);
        return -1;}
    dmaAddr = img_rdBuffer1.PhysicalAddr & 0xFFFFFFFC;  /* Set low read physical address */
    // Program DMA registers
    PldaMemoryWrite32 (nIndex, BAR_0, DMA1_RX_PHYADD_OFFSET,  1, &dmaAddr);
    PldaMemoryWrite32 (nIndex, BAR_0, DMA1_RX_SIZE_OFFSET,    1, (unsigned long*)&img_transferSize);

    // RESET PCI FIFOs reset dmas and fifos
    xpci_resetChannels(3);
    // set proper timeout
    xpci_setHardTimeout(HWTIMEOUT_1SEC);
    return 0;
}

// Function to check for exposition compatibility
//
// 12 bits images are used without overflow which limits the exposition time.
// With a flux of 10**6 photons/sec this makes 4msec before overflow.
// Note that in such condition using 16bits images will make possible exposition during 64msec
// which is onothet option.
// As a general rule we can consider that using 12 bits mode for exposition time higher than
// 100msec is probably not a good idea because the normal way to read on 16 or 32 bits is able
// to sustain such a frequency.
// So we need to advise the user.
// return: O OK
//         -1 KO
//
// ATTENTION: The IMXPAD systems are excluded from the test.  
//==============================================================================================
static int xpci_check12bitsConditions( int gateMode, int gateLength, int timeUnit){
    int maxExpose = 100; // in msec

    if((xpci_systemType==HUB)||(xpci_systemType==BACKPLANE)){
        if (gateMode!=0){ // cant check automatically external gate delay just advise the user
            printf("WARNING: You have choosen external trigger\n");
            printf("WARNING: Can't check automatically the expose time when controlled by external signals\n");
            printf("WARNING: This way to read images without overflow is not appropriate for expose delay longer than 10 msec\n");
            printf("WARNING: For expose delay longer than 10 msec use xpci_getOneImage() in a loop\n");
            return 0;
        }
        else { // check internal gate delay < 1sec
            if(    ( (timeUnit==2) && (gateLength>maxExpose) ) // millisec delay
                   ||   (timeUnit==3) )                   // sec delay
            {
                printf("ERROR: this images reading without overflow is not appropriate for expose delay longer than 10msec\n");
                printf("ERROR: For expose delay longer than 10msec use xpci_getOneImage() with complete 16bits image (with overflow)\n");
                return -1;
            }
        }
        return 0;
    }
    else{
        printf("WARNING: the expose time test does not apply to any of the IMXPAD systems\n");
        return 0;
    }
}

// Check conditions for fast image reading without overflow 12bits images.
// Function to check the fast reading conditions before initializing the fast
// reading sequence.
// Function to do expose and then read the data automatically at the expose end
// This function is used to activate fast image reading without overflow. The detector
// makes thus a faster acquisition. Nonetheless even if only 12 bits are usefull
// the data are received on 16 bits.
// 
// Note: We cant automatically check this condition if the gate is external.
// return 0 OK
//        else error
//
//==============================================================================
int xpci_getImageInit(enum IMG_TYPE type, int moduleMask, int nbChips,
                      int gateMode, int gateLength, int timeUnit){
    img_expose     = 1;
    img_gateMode   = gateMode;
    img_gateLength = gateLength;
    img_timeUnit   = timeUnit;
    img_gotImages  = 0;

    if (xpci_check12bitsConditions(gateMode,gateLength,timeUnit)!=0)
        return -1;
    return xpci_readImageInit(type, moduleMask, nbChips);
}


/****************************************************************************************
  Function to read the next image using the parameters that have been defined with the
  xpci_readImageInit() function.

  ATTENTION: the user doesnt access this function. it should use xpci_readNextImage(void *data)

  When using the data the user should cast on uint16_t or uint32_t depending on the type
of image currently read.

  INPUT: type   0[image]   1[flat config]
         data   pointer to the user memeory were data will be copied
  RETURN:       O success -1 error
*****************************************************************************************/
static int readNextImageStruct(enum DATA_TYPE type, void *data){ // dma transfer size in bytes
    int                   dmaStatus = 0;
    //int                   i,j;
    //int                   pline = 10;
    //uint16_t              *ptx, *dbgptx, *dbgptxU;
    //uint16_t              *img_data0, *img_data1;
    //uint16_t              *img_data0Start, *img_data1Start;
    //int flag=0;

    // First check in case exposition is requested that the timing conditions are correct
    // for image reading without overflow
    // This test does not apply to IMXPAD systems
    if((xpci_systemType==HUB)||(xpci_systemType==BACKPLANE)){
        if (img_expose==1){
            // Notice : case never tested because only called in xpci_getNextImage() never used
            // neither called by applications the 29/09/2011
            // combine expose + transfer but on 12 bits without overflow
            // check conditions are valid
            if (xpci_check12bitsConditions( img_gateMode, img_gateLength, img_timeUnit)!=0) {
                dmaStatus = -1; goto image_end;
            }
        }
    }

    // =================== REQUEST IMAGE ====================
    // Note: this way to send the request slows down the acquisition because
    //       memeory allocation for the DMA is done at each image for the
    //       command to send. If we want to optimize speed we will have to
    //       do allocation once for all in xpci_readImageInit() and just
    //       re-send the message at each loop.
    // We keep this way because it is more modular.

    xpci_it_pos  =1;
    switch(type){
    case IMG:
        if (debugMsg) printf("TYPE: Reading an image ...\n");
        if (img_type==B2){
            if(xpci_systemType==HUB){// version used with Hub system
                if (img_expose==1){ // conditions are valid because checked at the beginning
                    // Notice : case never tested because only called in 	xpci_getNextImage() never used
                    // neither called by applications the 29/09/2011
                    if (xpci_modImageGet2B(img_moduleMask, img_gateMode, img_gateLength, img_timeUnit) == -1){
                        printf("Error in expose+get image ... abort image reading\n");
                        dmaStatus = -1; goto image_end;
                    }
                }//expose
                else { //no expose direct transfer
                    if (xpci_modImage2BReq(img_moduleMask) == -1){
                        printf("Error in requesting image ... abort image reading\n");
                        dmaStatus = -1; goto image_end;
                    }
                }//expose
            }//HUB
            else if (xpci_systemType==BACKPLANE){ //BACKPLANE
                if (img_expose==1){ // combine expose + transfer
                    // Notice : case never tested because only called in 	xpci_getNextImage() never used
                    // neither called by applications the 29/09/2011
                    if (xpci_modImageGet2B_XPAD32(img_moduleMask, img_gateMode, img_gateLength, img_timeUnit, 1) == -1){
                        printf("\t%s(): Error in expose+get image ... abort image reading\n", __func__);
                        dmaStatus = -1; goto image_end;
                    }
                }//expose
                else { //no expose direct transfer
                    if (xpci_modImage2BReq(img_moduleMask) == -1){
                        printf("Error in requesting image ... abort image reading\n");
                        dmaStatus = -1; goto image_end;
                    }
                }//expose
            }//BACKPLANE
            else{// all IMXPAD systems
                // no exposure, direct image transfer
                if (xpci_modImage2BReq(img_moduleMask) == -1){
                    printf("Error in requesting image ... abort image reading\n");
                    dmaStatus = -1; goto image_end;
                }
            }
        }//B2

        else if (img_type==B4){
            if((xpci_systemType==HUB)||(xpci_systemType==BACKPLANE)){
                if (img_expose==1){ // combine expose + transfer but only for 12 bit images not valid in 4B
                    printf("\t%s():ERROR: 4B image request is not compatible with fast reading 12 bits images without overflow\n",__func__);
                    //if (xpci_modImageGet4B(img_moduleMask, img_gateMode, img_gateLength, img_timeUnit) == -1){
                    //printf("\t%s(): Error in expose+get image ... abort image reading\n", __func__);
                    dmaStatus = -1; goto image_end;
                }//expose
                else { //no expose direct transfer
                    if (xpci_modImage4BReq(img_moduleMask) == -1){
                        printf("Error in requesting image ... abort image reading\n");
                        dmaStatus = -1; goto image_end;
                    }
                }//expose
            }// HUB or BACKPLANE
            else{// all IMXPAD systems
                // no exposure, direct image transfer
                if (xpci_modImage4BReq(img_moduleMask) == -1){
                    printf("Error in requesting image ... abort image reading\n");
                    dmaStatus = -1; goto image_end;
                }
            }
        }//B4

        else {
            printf("\t%s()ERROR: image acquisition for unknown type %d\n", __func__,img_type);
            dmaStatus = -1; goto image_end;
        }
        break;

    case CONFIG:
        if (debugMsg) printf("TYPE: Reading a config ...\n");
        if (xpci_modReadConfig(img_moduleMask) == -1){
            printf("Error in requesting configuration ... abort image reading\n");
            dmaStatus = -1; goto image_end;
        }
        break;
    default:
        printf("ERROR: unknown type of image or pixel data to read\n");
        dmaStatus = -1; goto image_end;
    }
    xpci_it_pos  =2;
    // ================= READ ARRIVING IMAGE DATA ======================
    /* Parallel transfers on both channels are
     LOOP:
     Only first loop: read 0
     wait 0
     read 1
     copy 0 to mem
     wait 1
     Not in the last loop: read 0
     copy 1 to mem
     END
  */
    // read data flow
    dmaStatus =  xpci_readImgBuff(data, 0);

image_end:
    return dmaStatus;
}

/****************************************************************************************
  Function to provides parameters of the exposition and number of the images to be taken
  in the context of a fast reading (12 bits) images without overflow.

  ATTENTION: Hack in HUB config
             The second parameter gateMode is used to provide information about number
             of images (nloop) because gateMode is not used.

             The monomodule and backplane configs use one parameter more in the
             message to specify both gateMode and nloop

*****************************************************************************************/
int xpci_startImgSequence(unsigned nloop){
    int ret = 0;

    if((xpci_systemType==HUB)||(xpci_systemType==BACKPLANE)){
        if (img_type == B2){
            if (xpci_systemType==HUB){
                if (xpci_modImageGet2B(img_moduleMask, nloop, img_gateLength, img_timeUnit) == -1){
                    printf("\t%s():ERROR in sending request for 12bits images V31 reading ... abort image reading\n",__func__);
                    ret = -1; ;
                }
            }//HUB
            else { // BACKPLANE
                if (xpci_modImageGet2B_XPAD32(img_moduleMask, img_gateMode, img_gateLength, img_timeUnit, nloop) == -1){
                    printf("\t%s():ERROR in sending request for 12bits images V32 reading ... abort image reading\n",__func__);
                    ret = -1; ;
                }
            }//BACKPLANE
        }//B2
        else if (img_type == B4){
            /* useless better to protest to force the user to check what it is doing
     if (xpci_modImageGet4B(img_moduleMask, nloop, img_gateLength, img_timeUnit) == -1){
     printf("Error in expose+get image ... abort image reading\n");
     ret = -1; ;
      */
            printf("\t%s():ERROR: 4B image request is not compatible with fast reading 12 bits images without overflow\n",__func__);
            ret = -1;
        }//B4
        else {
            printf("\t%s():ERROR : unknown type of image to read\n",__func__);
            ret = -1;
        }
    }
    else{
        printf("\t%s():ERROR : function cannot be executed in any of the IMXPAD systems\n",__func__);
        ret = -1;





    }
    return ret;
}
/****************************************************************************************
  Function to read the PCIe buffers containing an image.
  returns: 0 success  else error
  input  : data      pointer to the data receiveing buffer
           timeout   maximum time in usec to wait on the IT
*****************************************************************************************/
int xpci_readImgBuff(void *data, int timeout){ // dma transfer size in bytes
    int                   dmaStatus = 0;
    int                   i,j;
    int                   pline = 5;
    uint16_t              *img_data0, *img_data1;
    uint16_t              *img_data0Start, *img_data1Start;
    int                   memPtr;
    int 				   copySize=(img_transferSize/sizeof(uint16_t));
    int 				  abortFlag=0;

    // compute start address to fill the reception buffer of each channels
    img_data0 = data;
    img_data0Start = img_data0;

    img_data1 = img_data0 + (img_sizeImage*img_nbMod0)/sizeof(uint16_t); //attention pointer inc of 2 bytes
    img_data1Start = img_data1;
    if (debugMsg){
        printf("transferSize %d sizeImage %d img_nbMod0 %d img_nbMod1 %d\n",
               img_transferSize,img_sizeImage,img_nbMod0,img_nbMod1 );
        printf("Start img_data0Start=0x%08x img_data1Start=0x%08x\n",
               (unsigned int)img_data0Start, (unsigned int)img_data1Start);
    }

    // ================= READ ARRIVING IMAGE DATA ======================
    /* Parallel transfers on both channels are
     LOOP:
     Only first loop: read 0
     wait 0
     read 1
     copy 0 to mem
     wait 1
     Not in the last loop: read 0
     copy 1 to mem
     END
  */

    // INFO at this point the FIFO 0 is already full and the FIFO 1 is near to be full !!!
    if (debugMsg) printf("Start parallel\n");
    initIt();
    for (i=1; i<=img_nbParallelTrans; i++){ // loop on parallel transfers

        if (i==1){ // only the first loop
            // If we have parallel transfers this is because we have at least one module on each
            // channel. We always start sith channel 0.
            if (debugMsg)
                printf("alternate DMA 0 ...\n");
            startDMARead(0);
            xpci_it_pos  =3;
        }//first loop

        do{
            WAIT_IT(dmaStatus,timeout);
            if(xpci_getAbortProcess()){
                abortFlag++;
                if (debugMsg) printf("abortFlag = %d\n",abortFlag);
            }
            if(xpci_getResetProcess())	break;
        }while(abortFlag==1 || abortFlag==2);        
		if(xpci_getResetProcess())	break;
		
        if(dmaStatus){
            goto image_end;
        }
        if (debugMsg) {
            printUserBuffer((unsigned*)img_rdBuffer0.UserAddr, pline );
            printf("alternate DMA 1 ...\n");
        }
        startDMARead(1);
        xpci_it_pos  =4;
        // copy previous buffer 0

        memcpy(img_data0, (uint32_t*)img_rdBuffer0.UserAddr ,img_transferSize);
        // for(memPtr=0;memPtr<copySize;memPtr++)
        // *(img_data0+memPtr) = *((uint16_t*)(img_rdBuffer0.UserAddr)+memPtr);

        img_data0 = img_data0+img_transferSize/sizeof(uint16_t);

        do{
            WAIT_IT(dmaStatus,timeout);
            if(xpci_getAbortProcess()){
                abortFlag++;
                if (debugMsg) printf("abortFlag = %d\n",abortFlag);
            }
            if(xpci_getResetProcess())	break;
        }while(abortFlag==1 || abortFlag==2);
		if(xpci_getResetProcess())	break;
		
        if(dmaStatus){
            goto image_end;
        }
        if (debugMsg) printUserBuffer((unsigned*) img_rdBuffer1.UserAddr, pline );

        if (i<img_nbParallelTrans){// not in lastloop - 1
            if (debugMsg) printf("alternate DMA 0 ...\n");
            startDMARead(0);
            xpci_it_pos  =5;
        }
        // copy previous buffer 1

        memcpy(img_data1,(uint32_t*)img_rdBuffer1.UserAddr ,img_transferSize);
        //for(memPtr=0;memPtr<copySize;memPtr++)
        //    *(img_data1+memPtr) = *((uint16_t*)(img_rdBuffer1.UserAddr)+memPtr);

        img_data1 = img_data1 + img_transferSize/sizeof(uint16_t);

    }//loop parallel
    

    /* do the remaining tranfers on the channel that have more modules to read
     FOR nbseqTrans
     start DMA on channel x
     wait copy data from channel x
  */
    if (debugMsg)
        printf("Sequencial ...\n");

    for (i=0; i<img_nbSeqTrans; i++){
        if (img_nbMod1<img_nbMod0){
            if (debugMsg)
                printf("sequencial DMA 0 ...\n");
            startDMARead(0);

            do{
                WAIT_IT(dmaStatus,timeout);
                if(xpci_getAbortProcess()){
                    abortFlag++;
                   if (debugMsg) printf("abortFlag = %d\n",abortFlag);
                }
                if(xpci_getResetProcess())	break;
            }while(abortFlag==1 || abortFlag == 2 );
            if(xpci_getResetProcess())	break;

            if(dmaStatus){
                goto image_end;
            }
            if (debugMsg) printUserBuffer((unsigned*) img_rdBuffer0.UserAddr, pline );

            // Bug seen at SOLEIL the 20-09-2011 with monomodule. Half images are skipped or overwritten
            // Changing the memory copy by a slower way to do it apparently solves the problem. To be
            // investigate in firmware.
            memcpy(img_data0, (uint32_t*)img_rdBuffer0.UserAddr ,img_transferSize);
            //for(memPtr=0;memPtr<(img_transferSize/sizeof(uint16_t));memPtr++)
            //    *(img_data0+memPtr) = *((uint16_t*)(img_rdBuffer0.UserAddr)+memPtr);

            img_data0 = img_data0+img_transferSize/sizeof(uint16_t); //attention pointer inc of 4 bytes);
        }
        else {
            if (debugMsg)
                printf("sequential DMA 1 ...\n");
            startDMARead(1);
            
            do{
                WAIT_IT(dmaStatus,timeout);
                if(xpci_getAbortProcess()){
                    abortFlag++;
                    if (debugMsg) printf("abortFlag = %d\n",abortFlag);
                }
                if(xpci_getResetProcess())	break;
            }while(abortFlag==1 || abortFlag == 2 );
            if(xpci_getResetProcess())	break;

            if(dmaStatus){
                goto image_end;
            }
            if (debugMsg) printUserBuffer((unsigned*)img_rdBuffer1.UserAddr, pline );

            // Bug seen at SOLEIL the 20-09-2011 with monomodule. Half images are skipped or overwritten
            // Changing the memory copy by a slower way to do it apparently solves the problem. To be
            // investigate in firmware.
            memcpy(img_data1, (uint32_t*)img_rdBuffer1.UserAddr ,img_transferSize);
            //for(memPtr=0;memPtr<(img_transferSize/sizeof(uint16_t));memPtr++)
            //    *(img_data1+memPtr) = *((uint16_t*)(img_rdBuffer1.UserAddr)+memPtr);

            //printf("current img_data1=0x%x\n",img_data1);
            img_data1 = img_data1+img_transferSize/sizeof(uint16_t); //attention pointer inc of 4 bytes);

        }
    }//loop sequential

image_end:
    return dmaStatus;
}

/***************************************************************************************
   Function to do fast reading of images without overflow (detector faster and only
   12 usefull bits). Nonetheless even if only 12 bits are usefull the data are received
   on 16 bits. Makes only sense if overflow not usefull expose <100ms.
   Function to acquire nloop images with the passed parameters and have them copied in
   the provided buffers.
   The nloop reception buffers should have been allocated before by the caller and their
   start addresses are passed as an array of addresses.

   IN  : type          type of image to get // only 2B is allowed
         moduleMask    mask of the module to read
         nbChips       nb of chips per modules
         nloop         nb of image to get
         pBuff         an array of pointers to each image reception buffer
         firstTimeout  maximum delay to wait for the first image reception
   OUT : the nloop image buffers contains the acquired images
   RET : 0 success  -1 error
**************************************************************************************/
int xpci_getImgSeq_CPPM(enum IMG_TYPE type, int moduleMask, int nbChips,
                        int gateMode, int gateLength, int timeUnit,
                        int nloop, void **pBuff, int firstTimeout){
    int i;

    //if (xpci_takeDetector()==-1){return -1;}

    img_gotImages = 0;
    // Checks parameters are compatible for 12 bits reading
    //  and initilizes the images reading process
    if(xpci_getImageInit(type, moduleMask, nbChips, gateMode, gateLength, timeUnit)==-1){
        printf("\t%s(): Fast reading (12bits) image acquisition init FAILED\n", __func__);
        goto failed;
    }

    // send command of
    xpci_hubReset();
    usleep(10);
    if(xpci_startImgSequence(nloop)==-1){
        printf("\t%s(): Fast reading (12bits) image acquisition request FAILED\n", __func__);
        goto failed;
    }

    for (i=0; i<nloop; i++){
        // sleep a bit to wait for long expose not to go in hardware timeout at the next reading
        // normally useless in fast reading where period should be far less than 8 img/sec
        xpci_delayWaitReply(gateLength, timeUnit);// normally useless in fast reading

        if (i==0){
            // start reading one the images in loop
            // for the first loop use a large timeout and a short for the others
            // set long timeout
            xpci_setHardTimeout(LONGER_TIMEOUT_HARD);//32
            if(xpci_readImgBuff( *(pBuff+i),firstTimeout )==-1 ){
                //if(xpci_readImgBuff( *(pBuff+i),0 )==-1 ){
                printf("\t%s(): image %d reading FAILED\n", __func__, i);
                goto failed;
            }
            // set proper short timeout for high speed
            xpci_setHardTimeout(TIMEOUT_HARD);//1sec
        }
        else {
            if(xpci_readImgBuff( *(pBuff+i), 0)==-1 ){
                printf("\t%s(): image %d reading FAILED\n", __func__, i);
                goto failed;
            }
        }

        img_gotImages++;

        // here we can put a break point in case an abort is requested
        // if ABORT
        //    send FE reset
        //    send PCIe reset (clean FIFO)
        //    break the loop
    }
    // release Plda ressources
    xpci_getImageClose();
    return 0;

failed:
    ////xpci_freeDetector();
    return -1;
}


// Function to know how many images have been already got in the images sequence
// returns: number of images
//==============================================================================
int   xpci_getGotImages(){
    return img_gotImages;
}

void xpci_readImageClose(){
    img_expose     =0; // set back to default which is read without expose
    PldaReleasePhysicalAddress(nIndex, DMA0_RX_BUF_LOCK);
    PldaReleasePhysicalAddress(nIndex, DMA1_RX_BUF_LOCK);
}

void xpci_getImageClose(){
    xpci_readImageClose();
}

// Function to read one single image
// data should be casted to uint16_t or uint32_t depending of IMG_TYPE
// CPPM implementation (original)
//=====================================================================
int   xpci_readOneImage_CPPM(enum IMG_TYPE type, int moduleMask, int nbChips, void *data){

    //if (xpci_takeDetector()==-1){return -1;}

    if (xpci_readImageInit(type, moduleMask, nbChips)!=0)
        goto failed;
    if (readNextImageStruct(IMG, data)!=0)
        goto failed;
    xpci_readImageClose();

   if(xpci_getAbortProcess()) return 1;
            
    return 0;

failed:
    //xpci_freeDetector();
    return -1;
}

// Function to read one single image
// data should be casted to uint16_t or uint32_t depending of IMG_TYPE
// imXPAD implementation (allocating complementary raw data buffer)
//=====================================================================
int   xpci_readOneImage_imxpad(enum IMG_TYPE type, int moduleMask, int nbChips, void *data){
    int      imgRawSize = 0;
    int      modNb = xpci_getModNb(moduleMask);
    uint16_t *pRawData;
    unsigned msgType = 0;
    int i=0;
    
    int ret;

    // allocate raw buffers depending on the type of the image
    if(type==B2){
        imgRawSize = 120*566*2*modNb;
        msgType = 1;
    }
    else{
        imgRawSize = 120*1226*2*modNb;
        msgType = 2;
    }
    xpix_imxpadWriteSubchnlReg(moduleMask, msgType, 1);

    pRawData = malloc(imgRawSize);
    
    memset(pRawData,0xbe,imgRawSize);

    ret = xpci_readOneImage_CPPM(type, moduleMask, nbChips, (void *)pRawData);
    if(ret == -1){
        free(pRawData);
        return -1;
    }

    if(type==B2){
        // allocate buffer for formatted 16 bits image
        if(imxpad_raw2data_16bits(moduleMask, pRawData, (uint16_t *)data)!=0){
            free(pRawData);
            return -1;
        }
    }
    else{
        // allocate buffer for formatted 32 bits image
        if(imxpad_raw2data_32bits(moduleMask, pRawData, (uint32_t *)data)!= 0){
            free(pRawData);
            return -1;
        }
    }
    return ret;
}

// Function to read one single image
// data should be casted to uint16_t or uint32_t depending of IMG_TYPE
// CPPM implementation (original)
//=====================================================================
int   xpci_readOneImage(enum IMG_TYPE type, int moduleMask, int nbChips, void *data){
    int ret = 0;
	xpci_clearResetProcess();
    ret = xpci_readOneImage_imxpad(type, moduleMask, nbChips, data);

    return ret;
}

// Function to read expose and then one single image
// data should be casted to uint16_t or uint32_t depending of IMG_TYPE
// returns    : 0 success    else error
// parameters : timeout (maximum timeout to wait in msec)
// ATTENTION: function is not compatible with IMXPAD systems
//=====================================================================
int   xpci_getOneImage(enum IMG_TYPE type, int moduleMask, int nbChips, void *data,
                       int gateMode, int gateLength, int timeUnit, int timeout){
    int ret;
    //if (xpci_takeDetector()==-1){return -1;}
    printf("Expose start\n");
    ret = xpci_modExpose(moduleMask,gateMode,gateLength,timeUnit,timeout);
    printf("Expose end with status %d\n", ret);
    if (ret!=0)
        goto failed;
    ret = xpci_readOneImage(type, moduleMask, nbChips, data);

    return ret;
failed:
    //xpci_freeDetector();
    return -1;
}

// Fonction to read the configuration registers currently loaded in the detector.
// data are received in the format of 2 bytes per pixel.
// bit[0] enable analog test
// bit[1] enable digital test
// bit[2] enable counters
// bit[8:3] DACL registers
// bit[15:9] reserved (tied to 0)
// in CPPM implementtion function returns a pointer to raw data
int xpci_getModConfig_CPPM(unsigned moduleMask, unsigned nbChips, uint16_t *data){
    if (xpci_readImageInit(B2, moduleMask, nbChips)==-1){
        printf("ERROR: %s() cant initialise for configuration reading\n", __func__);
        return -1;
    }
    if (readNextImageStruct(CONFIG, data)==-1){ // 1 is for config reading
        printf("ERROR: %s() cant receive configuration data from modules\n", __func__);
        return -1;
    }
    xpci_readImageClose();
    return 0;
}

// Fonction to read the configuration registers currently loaded in the detector.
// data are received in the format of 2 bytes per pixel.
// bit[0] enable analog test
// bit[1] enable digital test
// bit[2] enable counters
// bit[8:3] DACL registers
// bit[15:9] reserved (tied to 0)
// in imXPAD implementation function returns a pointer to image data 
int xpci_getModConfig_imxpad(unsigned moduleMask, unsigned nbChips, uint16_t *data){
    int      modNb = xpci_getModNb(moduleMask);
    uint16_t *pRawData;

    xpix_imxpadWriteSubchnlReg(moduleMask, 1, 1);

    pRawData = malloc(120*566*2*modNb);

    if(xpci_getModConfig_CPPM(moduleMask, nbChips, pRawData) != 0){
        free(pRawData);
        return -1;
    }


    if(imxpad_raw2data_16bits(moduleMask, pRawData, data)!=0){
        free(pRawData);
        return -2;
    }
    
    return 0;
}

// Fonction to read the configuration registers currently loaded in the detector.
// data are received in the format of 2 bytes per pixel.
int xpci_getModConfig(unsigned moduleMask, unsigned nbChips, uint16_t *data){ 
    int ret = 0;

    if((xpci_systemType==HUB)||(xpci_systemType==BACKPLANE))
        ret = xpci_getModConfig_CPPM(moduleMask, nbChips, data);
    else
        ret = xpci_getModConfig_imxpad(moduleMask, nbChips, data);

    return ret;

}

/**********************************************************
 * Function to do a command on a channel
***********************************************************/
static int xpci_doCommand(int channel, unsigned cmd){
    UINT32         cmdOffset,command;

    command = (unsigned long)cmd;
    switch(channel){
    case 0:
        cmdOffset = DMA0_CTRL;
        break;
    case 1:
        cmdOffset = DMA1_CTRL;
        break;
    default:
        cprintf("Unknown communication channel in xpci_doCommand()\n",  RED);
        return -1;
    }
    PldaMemoryWrite32 (nIndex, BAR_0, cmdOffset, 1 , &command);
    return 0;
}
// Local board reset aborts DMA, clear all fifos and registers
// active when "1"
// Should not be used by user that should use xpci_resetBoard();
//==============================================================
static int xpci_resetHardware(int det){
    int dum;

    unsigned long off = 0x0;
    unsigned long on  = 0x2; // bit 1 to 1
    PldaMemoryWrite32 (nIndex, BAR_0,PCIE_CMD_OFFSET, 1 , &off);
    dum = usleep(1);
    PldaMemoryWrite32 (nIndex, BAR_0,PCIE_CMD_OFFSET, 1 , &on);
    dum = usleep(1);
    PldaMemoryWrite32 (nIndex, BAR_0,PCIE_CMD_OFFSET, 1 , &off);
    dum = usleep(1);
    return 0;
}
// Abort cmd and do a reset of all FIFOs but dont change the registers
//=====================================================================
int  xpci_resetChannels(int channel){
    int ret, chn, dum;
    if (debugMsg) printf("Reseting channel %s(%d)\n",  __func__,channel);
    ret = 0;
    if (channel>=NB_CHANNELS){
        //all channels
        for (chn=0; chn<NB_CHANNELS; chn++)
            ret = ret + xpci_doCommand(chn, ABORT_TXDMA | ABORT_RXDMA | RESET_TXFIFO | RESET_RXFIFO);
    }
    else
        // one channel
        ret = xpci_doCommand(channel, ABORT_TXDMA | ABORT_RXDMA | RESET_TXFIFO | RESET_RXFIFO);
    dum = usleep(1); // experiments have proved it necesary otherwise the first IT following is lost
    if (ret!=0)
        return -1;
    else
        return 0;
}

// Abort cmd and do a reset of all FIFOs but dont change the registers
//=====================================================================
int  xpci_resetTXChannels(int channel){
    int ret, chn, dum;
    if (debugMsg) printf("Reseting channel %s(%d)\n",  __func__,channel);
    ret = 0;
    if (channel>=NB_CHANNELS){
        //all channels
        for (chn=0; chn<NB_CHANNELS; chn++)
            ret = ret + xpci_doCommand(chn, ABORT_TXDMA |  RESET_TXFIFO );
    }
    else
        // one channel
        ret = xpci_doCommand(channel, ABORT_TXDMA | RESET_TXFIFO );
    dum = usleep(1); // experiments have proved it necesary otherwise the first IT following is lost
    if (ret!=0)
        return -1;
    else
        return 0;
}








static int xpci_resetReadFifo(int channel){
    return xpci_doCommand(channel, RESET_RXFIFO);
}
static int xpci_resetWriteFifo(int channel){
    return xpci_doCommand(channel, RESET_TXFIFO);
}
static int xpci_abortRead(int channel){
    return xpci_doCommand(channel, ABORT_RXDMA);
}
static int xpci_abortWrite(int channel){
    return xpci_doCommand(channel, ABORT_TXDMA);
}
int xpci_setPCILoopMode(){
    unsigned long command = 0x1;
    PldaMemoryWrite32 (nIndex, BAR_0,PCIE_CMD_OFFSET, 1 , &command);
    return 0;
}
int xpci_resetPCILoopMode(){
    unsigned long command = 0x0;
    PldaMemoryWrite32 (nIndex, BAR_0,PCIE_CMD_OFFSET, 1 , &command);
    return 0;
}
/*************************************************************
 * Functions to get the service fifo message
**************************************************************/
int xpci_getDmaStatus(int channel, int print){
    uint8_t *add;

    switch(channel){
    case 0: add = (uint8_t *)(svcDma0.UserAddr); break;
    case 1: add = (uint8_t *)svcDma1.UserAddr; break;
    default: break;
    }
    // Attention bytes in 16 bits words are inverted
    // 0x52 0x2 0x20 0x58 0x69 0x74 0x65 0x6d 0x75 0x6f 0x0 0x74
    //   R   2   sp   X     i    t    e   m     u   o    O   t
    // means 2 RX timeout
    if (print){
        if( *(add+1)!=0){
            printf("Service fifo channel %d error status = %d message:", channel, *(add+1));
            printf("[0x%x]", *(add));

            for(int i = 2; i<SERV_BUFFER_SIZE; i=i+2)
            {
                //printf("%c%c", *(add+i+1),*(add+i));
                printf("%c%c", *(add+i),*(add+i+1));
                if (*(add+i+1)==0 || *(add+i)==0 )
                    break; //"\n" marks the end of the message
            }
            printf("\n");
            for(int i = 0; i<SERV_BUFFER_SIZE/sizeof(uint32_t); i++)
            {
                printf("Service fifo first word in svc fifo 0x%08x\n", add[i]);
            }
            printf("\n");

        }
        else
            printf("No error in service buffer\n");
    }
    return (int)(*(add+1));
}

/*************************************************************
 * Functions to look after the DMAs status registers
**************************************************************/
void  xpci_getStatusRegsTable(StatusRegTable **regs){
    PldaMemoryRead32(nIndex, BAR_0, 0, NB_STATUS_REGS, lastRegs.entry);
    *regs = &lastRegs;
}

void xpci_getFifoFillingStatus(int *level0, int *level1){
    StatusRegTable  *regs;
    int nb8bWords0, nb8bWords1;
    xpci_getStatusRegsTable(&regs);
    nb8bWords0 = regs->entry[10]          & 0xffff;
    nb8bWords1 = (regs->entry[10]  >> 15) & 0xffff;
    printf("RAW FIFO level 0x%08x\n", (int)regs->entry[10]);
    *level0 = nb8bWords0 << 3;
    *level1 = nb8bWords1 << 3;
}

int xpci_getInterruptStatus(){
    StatusRegTable *regs;
    xpci_getStatusRegsTable(&regs);
    return (int)(regs->entry[15] & 0x1);
}

void  xpci_dumpStatusRegsTable(){
    PldaMemoryRead32(nIndex, BAR_0, 0, NB_STATUS_REGS, lastRegs.entry);
    printf("Status registers table:\n");
    printf("offset 0  - TX0 address Ox%08x\n", (unsigned)lastRegs.entry[0]);
    printf("offset 1  - TX0 size    Ox%08x\n", (unsigned)lastRegs.entry[1]);
    printf("offset 2  - RX0 address Ox%08x\n", (unsigned)lastRegs.entry[2]);
    printf("offset 3  - RX0 size    Ox%08x\n", (unsigned)lastRegs.entry[3]);
    printf("offset 4  - TX1 address Ox%08x\n", (unsigned)lastRegs.entry[4]);
    printf("offset 5  - TX1 size    Ox%08x\n", (unsigned)lastRegs.entry[5]);
    printf("offset 6  - RX1 address Ox%08x\n", (unsigned)lastRegs.entry[6]);
    printf("offset 7  - RX1 size    Ox%08x\n", (unsigned)lastRegs.entry[7]);
    printf("offset 8  - SV0 address Ox%08x\n", (unsigned)lastRegs.entry[8]);
    printf("offset 9  - SV1 address Ox%08x\n", (unsigned)lastRegs.entry[9]);
    printf("offset 10 - RX FIFO status Ox%08x\n", (unsigned)lastRegs.entry[10]);
    printf("offset 11 - Firmware Code  Ox%08x\n", (unsigned)lastRegs.entry[11]);
    printf("offset 12 - Hard timeout   Ox%08x\n", (unsigned)lastRegs.entry[12]);
    printf("offset 15 - interrupt      Ox%08x\n", (unsigned)lastRegs.entry[15]);
    printf("xpciLib Version 3.0.7\n");
}
/*************************************************************
 * Functions to send command to the HUB
**************************************************************/
// Reset is sent on both channel, no reply is waited
//===================================================
static int xpci_resetPCItoHUBLinksFifo(){
    int ret = 0;
    ret = xpci_writeCommon(HUB_rst, sizeof(HUB_rst));
    usleep(100);
    return ret;
}

// add by fred for detector ALBA S1400 used reg Rev2 boards

static int xpci_regRstFifo(){
    int ret = 0;
    ret = xpci_writeCommon(REG_FIFO_rst, sizeof(REG_FIFO_rst));
    usleep(100);
    return ret;
}

static int xpci_regModuleName(){
    int ret = 0;
    ret = xpci_writeCommon(REG_Module_name, sizeof(REG_Module_name));
    usleep(100);
    return ret;
}



// For the HUB architecture this command resets the HUB FIFOs
int xpci_hubReset(){
    return xpci_resetPCItoHUBLinksFifo();
}
// For the bachplane architecture this command resets the GX FIFO in the PCIe board
int xpci_resetFifoPciGx(){
    return xpci_resetPCItoHUBLinksFifo();
}
int xpci_setHubLoopMode(){
    int ret = 0;
    int status = 0;
    int chnl;

    if (debugMsg) printf("Doing %s()\n", __func__);
    if (xpci_systemType!=HUB){
        printf("WARNING: only applicable to HUB system .. do nothing\n");
        return 0;
    }
    //for ( chnl=0; chnl<=1; chnl++){
    for ( chnl=1; chnl>=0; chnl--){
        ret = xpci_write(chnl, HUB_loop, sizeof(HUB_loop));
        if (ret) status = ret;
    }
    return status;
}
int xpci_resetHubLoopMode(){
    int ret = 0;
    int status = 0;
    int chnl;

    if (debugMsg) printf("Doing %s()\n", __func__);
    if (xpci_systemType!=HUB){
        printf("WARNING: only applicable to HUB system .. do nothing\n");
        return 0;
    }
    for ( chnl=0; chnl<=1; chnl++){
        ret = xpci_write(chnl, HUB_noLoop, sizeof(HUB_noLoop));
        if (ret) status = ret;
    }
    return status;
}
// Special for backplane version
// Depending on the system type differnet function is called
// Attention: in the IMXPAD systems, parameter 'mask' is ignored
int xpci_modRebootNIOS(unsigned mask){
		
	   xpci_setResetProcess();
       return  xpci_imxpadModRebootNIOS();
}

int xpci_imxpadModRebootNIOS(){
    int ret = 0;
    int status = 0;
    int chnl;

    if (debugMsg) printf("Doing %s\n",  __func__);
    initIt();
    for ( chnl=0; chnl<=1; chnl++){
        ret = xpci_write(chnl, MOD_imxpadRstNIOS, sizeof(MOD_imxpadRstNIOS));
        if (ret) status = ret;
    }
    if (status){
        printf("ERROR: %s() failed sending the request\n", __func__);
        return -1;
    }

    sleep(1);// NIOS boot is long ...
    return status;
}

int xpci_hubModRebootNIOS(unsigned mask){
    int ret = 0;
    int status = 0;
    int chnl;

    if (debugMsg) printf("Doing %s(0x%04x)\n",  __func__,mask);
    HUB_rebootModNIOS[6] = (uint16_t)mask;
    initIt();
    for ( chnl=0; chnl<=1; chnl++){
        ret = xpci_write(chnl, HUB_rebootModNIOS, sizeof(HUB_rebootModNIOS));
        if (ret) status = ret;
    }
    if (status){
        printf("ERROR: %s() failed sending the request\n", __func__);
        return -1;
    }
    sleep(1);// NIOS boot is long ...
    return status;
}

// Only available for HUB
int xpci_hubModReconfFPGA(unsigned mask){
    int ret = 0;
    int status = 0;
    int chnl;

    if (xpci_systemType!=HUB){

        printf("ATTENTION: xpci_hubModReconfFPGA() is only available for system with the HUB board\n");
        return 0;
    }
    if (debugMsg) printf("Doing %s(0x%04x)\n",  __func__,mask);
    HUB_reconfModFPGA[6] = (uint16_t)mask;
    initIt();
    for ( chnl=0; chnl<=1; chnl++){
        ret = xpci_write(chnl, HUB_reconfModFPGA, sizeof(HUB_reconfModFPGA));
        if (ret) status = ret;
    }
    if (status){
        printf("ERROR: %s() failed sending the request\n" ,__func__);
        return -1;
    }
    printf("Wait 2sec for FPGA reloading\n");
    sleep(2);// FPGA loading is long ...
    xpci_hubReset();
    xpci_resetBoard(0);
    return status;
}
int xpci_SetHub2bImages(){
    int ret = 0;
    int status = 0;
    int chnl;

    if (debugMsg) printf("Doing %s()\n", __func__);
    for ( chnl=0; chnl<=1; chnl++){
        ret = xpci_write(chnl, HUB_2bImgMode, sizeof(HUB_2bImgMode));
        if (ret) status = ret;
    }
    return status;
}

/********************************************************************
  Function to check that data received from one module are OK and
  not noise by checking the message structure.
  RETURN : 1 OK
           0 KO
*********************************************************************/
static int isAskReadyReplyOK(uint16_t *rcvData, int *moduleNb){
    *moduleNb=0;

    if ( (rcvData[0]==MOD_HEADER) &&
         (rcvData[1]!=0) &&
         (rcvData[2]==NB_WORDS_REPLY) &
         (rcvData[3]==MOD_REQ_READY_ACK) &&
         (rcvData[15]==MSG_TRAILER)){
        switch ( xpci_systemType){
        case HUB:
        case IMXPAD_S70:
        case IMXPAD_S140:
        case IMXPAD_S340:
        case IMXPAD_S540:
        case IMXPAD_S700:
            *moduleNb = rcvData[1] & 0xf;
            break;
        case IMXPAD_S420:
        case BACKPLANE :
            *moduleNb = rcvData[1]&0xff;
            break;
        case IMXPAD_S1400:
            *moduleNb = rcvData[1] & 0x1f;
            break;
        default : printf("ERROR :DETECTOR TYPE \n");
        }
        if (debugMsg) printf("ModuleNb = %d\tDay = 0x%x\tMonth = 0x%x\tYear = 0x%x\n",rcvData[1],rcvData[4],rcvData[5],rcvData[6]);
        FirmwareID = (rcvData[4] & 0xFF) << 24 | (rcvData[5] & 0xFF) << 16 | (rcvData[6] & 0xFFFF);
        return 1;
    }
    else {
        printf("ERROR: %s() Bad reply format to ask ready\n", __func__);
        return 0;
    }
}
// Function to scan all modules with 'ask ready' to test if they are connected
// and build the module mask from the answers
// out : inMask (the modules mask)
// returns : 0 success, -1 error
//----------------------------------------------------------------------------
int xpci_modAskReady(unsigned *inMask){
    int i, chnl, module, targetMask, source;
    int ret = 0;
    struct ModuleReplyBuffer rcvBuf;
    uint16_t* msg;
    int modules_nb = 1;
    int start_module = 0;

    if (debugMsg) printf("Doing %s()\n",  __func__);
    //if (xpci_takeDetector()==-1){return -1;}

    xpci_resetBoard(0);
    usleep(100);

    // different imxpad systems consist in different number of modules
    switch(xpci_systemType){
    case IMXPAD_S70:
        modules_nb = 1;
        start_module = 0;
        break;
    case IMXPAD_S140:
        modules_nb = 2;
        start_module = 0;
        break;
    case IMXPAD_S340:
        modules_nb = 5;
        start_module = 0;
        break;
    case IMXPAD_S420:
        modules_nb = 6;
        start_module = 2;
        break;
    case IMXPAD_S540:
    case HUB:
        modules_nb = 8;
        start_module = 0;
        break;
    case IMXPAD_S1400: 
        modules_nb = 20;
        start_module = 0;
        xpci_regRstFifo();
        xpci_regModuleName();
        break;
    case IMXPAD_S700: 
        modules_nb = 10;
        start_module = 0;
        xpci_regRstFifo();
        xpci_regModuleName();
        break;
    default:
        modules_nb = 8;

        start_module = 0;
        break;
    }



    if (debugMsg)printf("xpci_systemType %d\n",xpci_systemType);
    if (debugMsg)printf("modules number %d\n",modules_nb);
    *inMask=0;

    switch ( xpci_systemType){
    case IMXPAD_S70:
    case IMXPAD_S140:
    case IMXPAD_S340:
    case IMXPAD_S420:
    case IMXPAD_S540:
    case IMXPAD_S700:
    case IMXPAD_S1400:
    {
        for (module=start_module; module<(start_module + modules_nb); module++){
            targetMask = 1<<module;
            if (xpci_systemType== IMXPAD_S1400){
                chnl = CHANNEL_S1400(module);
                if(targetMask > 0x03ff)
                      targetMask = (targetMask >> 10) | 0x0400;
                else 
					  targetMask = targetMask;
            }
            else if (xpci_systemType== IMXPAD_S700){
                chnl = CHANNEL_S700(module);
            }
            else{
                chnl = CHANNEL(module);
            }
            if (debugMsg) printf("\n\nTest liveliness of module %d on channel %d target mask 0x%04x\n\n", module+1, chnl, targetMask);
            // configure subchannel registers
            xpix_imxpadWriteSubchnlReg(1<<module, 0, 1);
            msg = malloc(sizeof(MOD_askReady));
            memcpy(msg,MOD_askReady,sizeof(MOD_askReady));
            msg[3] = targetMask; // set the mask value

            if (debugMsg) printf(">>>>>> Send Ask Ready to module %d<<<<<<\n", module+1);
            ret = xpci_write(chnl, msg, sizeof(MOD_askReady));
            free(msg);
            if (ret==-1) {
                printf("ERROR: %s() failed sending the request to modules nb %d\n",  __func__, module+1);
                return -1;
            }
            memset(&rcvBuf,0,sizeof(struct ModuleReplyBuffer));
            if (debugMsg) printf(">>>>>> Get reply from  Ask Ready module %d<<<<<<\n", module+1);

            ret = xpci_read(chnl, (uint16_t*)rcvBuf.data, MOD_REPLY_SIZE, 1000);
            
            if (ret==-1) {
                printf("ERROR: %s() failed reading the reply on module %d\n",  __func__, module+1);
            }
            else {
                if (isAskReadyReplyOK((uint16_t*)rcvBuf.data, &source)!=1){
					//FirmwareID = 
                    continue;
                }
                if (source != (module+1)){
                    printf("ERROR: module %d is not on the proper channel: check connections\n",source);
                    // return -1;
                }
                *inMask = *inMask | (1<<module);
                if(debugMsg){
                    printf("REPLY %s() from module %d: \n",  __func__, module+1);
                    for (i=0; i<MOD_REPLY_SIZE; i++){
                        printf("0x%08x ", (int)rcvBuf.data[i]);
                    }
                    printf("\n");
                }//debug
            }//ret
        }//modules
        break; // end case HUB
    }
    default:
        printf("ERROR: UNKNOWN detector type\n");
        return -1;
    }
    //xpci_freeDetector();

    return 0;
}

// *******************************************************************************
// added by imXPAD
// This function that sends ask ready to multiple modules and 
// verifies if all modules has answered
// *******************************************************************************
int xpci_modGlobalAskReady(unsigned modMask){
    int i, module, readMask, source;
    int modules_nb = xpci_getModNb(modMask);
    int lastMod    = xpci_getLastMod(modMask);
    int FirstMod   = xpci_getFirstMod(modMask);
    int modEnable = 0;
    int ret = 0;
    unsigned readData[modules_nb*16];
    //unsigned *readData;
    uint16_t retAskReady[16];
    //uint16_t *retAskReady;
    uint16_t *msg;

    if (xpci_systemType == IMXPAD_S1400 || xpci_systemType == IMXPAD_S700){
        xpci_regRstFifo();
    }

    xpix_imxpadWriteSubchnlReg(modMask, 0, 1);
   // xpix_imxpadWriteSubchnlReg(modMask, 0, 1);
    // send Ask Ready command
    msg = malloc(sizeof(MOD_askReady));
    memcpy(msg,MOD_askReady,sizeof(MOD_askReady));
    msg[3] = modMask; // set the mask value
    //  ret = xpci_writeCommon(msg, sizeof(MOD_askReady));
    if(xpci_systemType == IMXPAD_S1400){
        ret = xpci_writeCommon_S1400(msg, sizeof(MOD_askReady),modMask);
    }
    else{
        ret = xpci_writeCommon(msg, sizeof(MOD_askReady));
    }

    free(msg);
    if (ret==-1) {
        printf("ERROR: %s() failed sending the request to modules nb %d\n",  __func__, module+1);
        return -1;
    }
    // allocate read buffer
    //readData = malloc(modules_nb*MOD_REPLY_SIZE);
    //retAskReady = malloc(MOD_REPLY_SIZE);

    ret = waitCommandReplyExtended(modMask, (char*)__func__, 5000, readData);
    if (ret==-1) {
        //printf("ERROR: %s() failed reading the reply from modules 0x%x\n",  __func__, modMask);
        //free(readData);
        //free(retAskReady);
        return -1;
    }
    else {
        readMask = 0;
        for(module=FirstMod; module<lastMod+1; module++){
            if(modMask & (1 << module))
            {
                for(i=0; i<16; i++)
                    retAskReady[i]=readData[16*modEnable+i];

                if (isAskReadyReplyOK(retAskReady, &source)!=1)
                    continue;
                readMask = readMask | (1<<(readData[16*modEnable+1]-1));
                modEnable++;
            }
        }
        if (readMask!=modMask){
            return -1;
        }

    }
    return 0;
}


//======================================================================
// ATTENTION -> added by Arek !!!
// function to send one AskReady command to slected module
//======================================================================
int xpci_modSingleAskReady(int module){
    int i, chnl, targetMask;
    int ret = 0;
    struct ModuleReplyBuffer rcvBuf;
    uint16_t* msg;

    if (debugMsg) printf("Doing %s()\n",  __func__);

    //if (xpci_takeDetector()==-1){return -1;}
    if (xpci_systemType == IMXPAD_S1400){
        xpci_regRstFifo();
        chnl = CHANNEL_S1400(module);
    }
    else if (xpci_systemType == IMXPAD_S700){
        xpci_regRstFifo();
        chnl = CHANNEL_S700(module);
    }
    else{
        chnl = CHANNEL(module);
    }
    targetMask = 1<<module;

    // configure subchannel registers
    xpix_imxpadWriteSubchnlReg(targetMask, 0, 1);

    msg = malloc(sizeof(MOD_askReady));
    memcpy(msg,MOD_askReady,sizeof(MOD_askReady));
    msg[3] = targetMask; // set the mask value
    for (i=0; i<=5; i++)
        printf("0x%04x ", (unsigned int)msg[i]);
    printf("\n");

    if (debugMsg) printf(">>>>>> Send Ask Ready to module %d<<<<<<\n", module+1);
    ret = xpci_write(chnl, msg, sizeof(MOD_askReady));
    free(msg);
    if (ret==-1) {
        printf("ERROR: %s() failed sending the request to modules nb %d\n",  __func__, module+1);
        //xpci_freeDetector();
        return -1;
    }
    memset(&rcvBuf,0,sizeof(struct ModuleReplyBuffer));
    if (debugMsg) printf(">>>>>> Get reply from  Ask Ready module %d<<<<<<\n", module+1);
    ret = xpci_read(chnl, (uint16_t*)rcvBuf.data, MOD_REPLY_SIZE, 500);
    if (ret==-1) {
        printf("ERROR: %s() failed reading the reply on module %d\n",  __func__, module+1);
        //xpci_freeDetector();
        return -1;
    }
    else {
        if(debugMsg){
            printf("REPLY %s() from module %d: \n",  __func__, module+1);
            for (i=0; i<MOD_REPLY_SIZE; i++){
                printf("0x%08x ", (int)rcvBuf.data[i]);
            }
            printf("\n");
        }//debug
    }//ret
    //xpci_freeDetector();
    return 0;
}
// Function to collect the reply messages from a command on all channels
// returns: 0 succes    else error
// params : modMask     modules mask
//          userFunc    calling function name for debug
//          timeout     maximum delay in ms
//======================================================================
int waitCommandReply(unsigned modMask, char *userFunc, int timeout){
    struct ModuleReplyBuffer rcvBuf[10];
    int i,j, chnl, nbMod;
    int ret = 0;
    uint16_t *rptr;

    memset(rcvBuf,0,sizeof(rcvBuf));
    for ( chnl=0; chnl<=1; chnl++){
        if (!isChannelUsed(chnl, modMask))
            continue;
        nbMod = nbModOnChannel(chnl, modMask);
        ret = xpci_read(chnl, (uint16_t*)rcvBuf[chnl].data, MOD_REPLY_SIZE*nbMod, timeout);
        if (ret) {
            printf("ERROR: %s()/%s()  failed reading the reply on %d\n", __func__,userFunc,chnl);
            return -1;
        }
        if(debugMsg)
        {
            printf("REPLY %s()/%s() channel %d: \n",__func__,userFunc, chnl);
            rptr = (uint16_t*)(rcvBuf[chnl].data);
            for (j=0;j<nbMod;j++)
            {
                for (i=0; i<(MOD_REPLY_SIZE/sizeof(uint16_t)); i++){
                    printf("0x%04x ", rptr[i+j*16]);
                }
                printf("\n");
            }
            printf("\n");
        }//debug
    }//for chnl
    return 0;
}

// Loads a known value in the modules counters that can be read later as an
// image and checked.
//============================================================================
int xpci_modLoadAutoTest(unsigned modMask, unsigned value, unsigned mode){
    int ret = 0;
    uint16_t *msg;
    unsigned Mask =0;

    switch(xpci_systemType){
    case HUB:
    case IMXPAD_S70:
    case IMXPAD_S140:
    case IMXPAD_S340:
    case IMXPAD_S420:
    case IMXPAD_S540:
    case IMXPAD_S700:
    case IMXPAD_S1400:
        Mask = modMask;break;
    case BACKPLANE:
        Mask = modMask & 0x55;break;
    default:
        printf("ERROR: UNKNOWN detector type\n");
        return -1;
    }

    if (debugMsg) printf("Doing %s(0x%04x, %d, %d)\n",  __func__, Mask,value,mode);
    //if (xpci_takeDetector()==-1){return -1;}

    if ( Mask==0)
        return 0;

    // configure subchannel registers
    xpix_imxpadWriteSubchnlReg(Mask, 0, 1);

    msg = malloc(sizeof(MOD_autoTest));
    memcpy(msg,MOD_autoTest,sizeof(MOD_autoTest));
    msg[3] = Mask; // set the mask value
    msg[8] = value; // set the counter value
    msg[9] = mode;

    if(xpci_systemType == IMXPAD_S1400){
        ret = xpci_writeCommon_S1400(msg, sizeof(MOD_autoTest),modMask);
    }
    else{
        ret = xpci_writeCommon(msg, sizeof(MOD_autoTest));
    }
    free(msg);
    if (ret){
        printf("ERROR: %s() failed sending the request\n" ,__func__);
        return -1;
    }
    // Now wait to receive ACK message with one message on each channel if necessary
    // We dont check all modules ... only one per channel
    xpci_setHardTimeout(HWTIMEOUT_8SEC); // set a long hardware timeout 8s for this function
    printf("INFO: %s() ... WAIT ...can be long for the data to load ...\n", __func__);
    //sleep(10); // ATTENTION REALLY NECESSARY

    ret = waitCommandReply(Mask, (char*)__func__, 1500000);

    xpci_setHardTimeout(HWTIMEOUT_1SEC);//set back default value
    //xpci_freeDetector();
    return ret;
}
// Configuration of the selected register in the selected chip.
// Loads the general config value "regVal" in the register "reg" of the module.
//==============================================================================
int xpci_modLoadConfigG(unsigned modMask, unsigned chipMask, unsigned reg, unsigned regVal){

    int      ret = 0;
    uint16_t *msg;
    unsigned Mask=0;
    unsigned selchip=0;
    int      module, mask;
    int start_module = 0;
    int modules_nb = xpci_getModNb(modMask);
    int lastMod    = xpci_getLastMod(modMask);
    int FirstMod   = xpci_getFirstMod(modMask);

    // different imxpad systems consist in different number of modules
    //   xpadModuleType( &modules_nb , &start_module);

    if(modMask==0)  return -1;

    for (module=FirstMod; module<lastMod; module++){
        mask = modMask & (1<<module);
        if(mask == 0)
            continue;
        if (mask){

            switch(xpci_systemType){
            case HUB:
            case IMXPAD_S70:
            case IMXPAD_S140:
            case IMXPAD_S340:
            case IMXPAD_S420:
            case IMXPAD_S540:
            case IMXPAD_S700:
            case IMXPAD_S1400:
                Mask = mask;
                selchip = chipMask;
                break;
            default:
                printf("ERROR: UNKNOWN detector type\n");
                return -1;
            }

            if (debugMsg) printf("Doing %s(0x%04x, 0x%04x, %d, %d)\n",  __func__, Mask,selchip,reg,regVal);
            //if (xpci_takeDetector()==-1){return -1;}


            // configure subchannel registers
            xpix_imxpadWriteSubchnlReg(Mask, 0, 1);

            msg = malloc(sizeof(MOD_configG));
            memcpy(msg,MOD_configG,sizeof(MOD_configG));
            msg[3]  = (uint16_t)Mask; // set the mask value
            msg[8] = (uint16_t)selchip;
            msg[9] = (uint16_t)reg;
            msg[10] = (uint16_t)regVal;

            //  ret = xpci_writeCommon(msg, sizeof(MOD_configG));
            if(xpci_systemType == IMXPAD_S1400){
                ret = xpci_writeCommon_S1400(msg, sizeof(MOD_configG),Mask);
            }
            else{
                ret = xpci_writeCommon(msg, sizeof(MOD_configG));
            }
            free(msg);
            if (ret){
                printf("ERROR: %s() failed sending the request\n", __func__);
                //xpci_freeDetector();
                return -1;
            }
            // Now wait to receive ACK for this loading
            //xpci_freeDetector();

            ret = waitCommandReply(Mask, (char*)__func__, 15000);
            if (ret!=0)
                return ret;
        }//mask

    }//module
    return ret;
}
// Configuration of all general registers in the selected chip.
//==============================================================================
int xpci_modLoadAllConfigG(unsigned modMask, unsigned chipMask,
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
                           ){

    int        ret = 0;
    uint16_t   *msg;
    unsigned   Mask=0;
    unsigned   selchip=0;
    unsigned   module, mask;
    // different imxpad systems consist in different number of modules
    //  xpadModuleType( &modules_nb , &start_module);

    int modules_nb = xpci_getModNb(modMask);
    int lastMod    = xpci_getLastMod(modMask);
    int FirstMod   = xpci_getFirstMod(modMask);

    for (module=FirstMod; module<=lastMod; module++){
        mask = modMask & (1<<module);
        if(Mask == 0) continue;
        if (mask){
            switch(xpci_systemType){
            case HUB:
            case IMXPAD_S70:
            case IMXPAD_S140:
            case IMXPAD_S340:
            case IMXPAD_S420:
            case IMXPAD_S540:
            case IMXPAD_S700:
            case IMXPAD_S1400:
                Mask = mask;
                selchip = chipMask;break;
            default:
                printf("ERROR: UNKNOWN detector type\n");
                return -1;
            }
            if (debugMsg) printf("Doing %s(0x%04x, 0x%04x, ... )\n",  __func__,Mask,chipMask);
            if (Mask==0)
                return 0;

            // configure subchannel registers
            xpix_imxpadWriteSubchnlReg(Mask, 0, 1);

            msg = malloc(sizeof(MOD_allConfigG));
            memcpy(msg,MOD_allConfigG,sizeof(MOD_allConfigG));
            msg[3]  = Mask; // set the mask value
            msg[8] = selchip;
            msg[9] = cmosVal;
            msg[10] = amptpVal;
            msg[11] = ithhVal;
            msg[12] = vadjVal;
            msg[13] = vrefVal;
            msg[14] = imfpVal;
            msg[15] = iotaVal;
            msg[16] = ipreVal;
            msg[17] = ithlVal;
            msg[18] = ituneVal;
            msg[19] = iBuffer;
            // ret = xpci_writeCommon(msg, sizeof(MOD_allConfigG));
            if(xpci_systemType == IMXPAD_S1400){
                ret = xpci_writeCommon_S1400(msg, sizeof(MOD_allConfigG),mask);
            }
            else{
                ret = xpci_writeCommon(msg, sizeof(MOD_allConfigG));
            }
            free(msg);
            if (ret){
                printf("ERROR: %s() failed sending the request\n", __func__);
                //xpci_freeDetector();
                return -1;
            }

            // Now wait for ACK on this loading
            xpci_setHardTimeout(HWTIMEOUT_8SEC); // set a long hardware timeout 8s for this function
            ret = waitCommandReply(Mask, (char*)__func__, 15000);
            if (ret!=0)
                return -1;
        }//mask
    }//module
    //xpci_freeDetector();
    xpci_setHardTimeout(HWTIMEOUT_1SEC);
    return ret;
}
// Loads the same "value" in the DACL register in the selected module's chip.
//===========================================================================
int xpci_modLoadFlatConfig(unsigned modMask, unsigned chipMask, unsigned value){
    int      ret = 0;
    uint16_t *msg;
    unsigned Mask=0;

    switch(xpci_systemType){
    case HUB:
    case IMXPAD_S70:
    case IMXPAD_S140:
    case IMXPAD_S340:
    case IMXPAD_S420:
    case IMXPAD_S540:
    case IMXPAD_S700:
    case IMXPAD_S1400:
        Mask = modMask;break;
    default:
        printf("ERROR: UNKNOWN detector type\n");
        return -1;
    }

    if (debugMsg) printf("Doing %s(0x%04x, 0x%04x, %d)\n",  __func__,Mask,chipMask,value);
    if ( Mask==0)
        return 0;

    // configure subchannel registers
    xpix_imxpadWriteSubchnlReg(Mask, 0, 1);

    msg = malloc(sizeof(MOD_flatConfig));
    memcpy(msg,MOD_flatConfig,sizeof(MOD_flatConfig));
    msg[3]  = Mask; // set the mask value
    msg[9] = chipMask;
    msg[10] = value;

    // ret = xpci_writeCommon(msg, sizeof(MOD_flatConfig));
    if(xpci_systemType == IMXPAD_S1400){
        ret = xpci_writeCommon_S1400(msg, sizeof(MOD_flatConfig),Mask);
    }
    else{
        ret = xpci_writeCommon(msg, sizeof(MOD_flatConfig));
    }
    free(msg);
    if (ret){
        printf("ERROR: %s() failed sending the request\n", __func__);
        //xpci_freeDetector();
        return -1;
    }
    // printf("%s() WAIT 15 sec for the data to load\n", __func__);
    // sleep(15); // ATTENTION REALLY NECESSARY
    ret = waitCommandReply(Mask, (char*)__func__, 150000);
    //xpci_freeDetector();
    return ret;
}
// Command diffusing a read image command to all existing modules
//===============================================================
int xpci_modImage2BReq(unsigned modMask){
    uint16_t *msg;
    int       ret;

    if (debugMsg) printf("Doing %s(0x%04x)\n",  __func__,modMask);
    //NO slows down the acquisition speed and not usefull !!! xpci_hubReset();
    if ( modMask==0)
        return 0;
    msg = malloc(sizeof(MOD_img2B));
    memcpy(msg,MOD_img2B,sizeof(MOD_img2B));
    msg[3]  = (uint16_t)modMask; // set the mask value
    if(xpci_systemType == IMXPAD_S1400){
        ret = xpci_writeCommon_S1400(msg, sizeof(MOD_img2B),modMask);
    }
    else{
        ret = xpci_writeCommon(msg, sizeof(MOD_img2B));
    }
    
    free(msg);
    return ret;
}
// Command diffusing a read image command to all existing modules
//===============================================================
int xpci_modImage4BReq(unsigned modMask){
    uint16_t *msg;
    int       ret;

    if (debugMsg) printf("Doing %s(0x%04x)\n",  __func__,modMask);
    //NO slows down the acquisition speed and not usefull !!! xpci_hubReset();
    if ( modMask==0)
        return 0;
    msg = malloc(sizeof(MOD_img4B));
    memcpy(msg,MOD_img4B,sizeof(MOD_img4B));
    msg[3]  = (uint16_t)modMask; // set the mask value
    //ret = xpci_writeCommon(msg, sizeof(MOD_img4B));
    if(xpci_systemType == IMXPAD_S1400){
        ret = xpci_writeCommon_S1400(msg, sizeof(MOD_img4B),modMask);
    }
    else{
        ret = xpci_writeCommon(msg, sizeof(MOD_img4B));
    }
    free(msg);
    return ret;
}
// Fast version for use in the readImageNext() function
// where speed has to be optimized
//======================================================
// NON UTILSE
int xpci_modImage2BReqNextImage(unsigned modMask){
    uint16_t *msg;
    int       ret;

    if((xpci_systemType==HUB)||(xpci_systemType==BACKPLANE)){
        if (debugMsg) printf("Doing %s(0x%04x)\n",  __func__,modMask);
        //NO slows down the acquisition speed and not usefull !!! xpci_hubReset();
        if ( modMask==0)
            return 0;
        msg = malloc(sizeof(MOD_img2B));
        memcpy(msg,MOD_img2B,sizeof(MOD_img2B));
        msg[3]  = (uint16_t)modMask; // set the mask value
        // select the fast function using preallocated buffers
        ret = xpci_writeCommonNextImage(msg, sizeof(MOD_img2B));
        free(msg);
    }
    else{
        printf("ERROR: %s() function does not apply to any of the IMXPAD systems\n", __func__);
        ret = -1;
    }
    return ret;
}
// Send a command to read the configuration registers values from the detector and tranfser to the
// PC memeory (like an image) in the format of 16bits per pixel.
// bit[0] = enable analog test
// bit[1] = enable digital test
// bit[2] = enable counters test
// bit[3:8] = DACL registers
// bit[9:15] = reserved ttied to 0)
//====================================================================================
int xpci_modReadConfig(unsigned modMask){
    int       status = 0;
    uint16_t *msg;

    if (debugMsg) printf("Doing %s(0x%04x)\n",  __func__,modMask);
    xpci_hubReset();
    if ( modMask==0)
        return 0;
    msg = malloc(sizeof(MOD_readConfig));
    memcpy(msg,MOD_readConfig,sizeof(MOD_readConfig));
    msg[3]  = (uint16_t)modMask; // set the mask value

    // status = xpci_writeCommon(msg, sizeof(MOD_readConfig));

    if(xpci_systemType == IMXPAD_S1400){
        status = xpci_writeCommon_S1400(msg, sizeof(MOD_readConfig),modMask);
    }
    else{
        status = xpci_writeCommon(msg, sizeof(MOD_readConfig));
    }


    free(msg);
    if (status){
        printf("ERROR: %s() failed sending the request\n", __func__);
        return -1;
    }
    else
        return 0;
}

// Calibration data (80 words 16 bits) for one row of one chip (9 bits)
// Copy in the module memory buffer "calibId" at line "curRow" of chip "chipId" the 80 calibration data 
// values that are stored starting at address "value".
// ATTENTION: only one chip of one module at a time
//====================================================================================================
int   xpci_modSaveConfigL(unsigned modMask, unsigned calibId, unsigned chipId, unsigned curRow, unsigned *value){
    int          i, pos, module, mask;
    int          ret = 0;
    uint16_t     *msg;
    unsigned     Mask=0;
    unsigned int selchip=0;
    int 	 start_module = 0;

    int modules_nb = xpci_getModNb(modMask);
    int lastMod    = xpci_getLastMod(modMask);
    int FirstMod   = xpci_getFirstMod(modMask);


    // different imxpad systems consist in different number of modules
    //    xpadModuleType( &modules_nb , &start_module);

    if(modMask == 0)
        return 0;

    for(module=FirstMod;module<=lastMod;module++)
    {
        mask = modMask & (1 << module);
        if(mask==0)
            continue;
        switch(xpci_systemType){
        case HUB:
        case IMXPAD_S70:
        case IMXPAD_S140:
        case IMXPAD_S340:
        case IMXPAD_S420:
        case IMXPAD_S540:
        case IMXPAD_S700:
        case IMXPAD_S1400:
            Mask = mask;
            selchip = chipId;break;
        default: printf("ERROR: UNKNOWN detector type\n");
            return -1;
        }
        if (debugMsg) printf("Doing %s(0x%04x, 0x%04x, 0x%04x, ... )\n",  __func__,Mask, calibId, selchip);
        if ( modMask==0)
            return 0;

        // configure subchannel registers
        xpix_imxpadWriteSubchnlReg(Mask, 0, 1);

        msg = malloc(sizeof(MOD_saveConfigL));
        memcpy(msg,MOD_saveConfigL,sizeof(MOD_saveConfigL));
        msg[3]  = Mask;   // set the mask value
        msg[8]  = calibId;
        msg[9]  = selchip;   // current chip
        msg[10] = curRow;   // current row
        pos = 11;
        for (i=0; i<80; i++){
            msg[pos+i]=value[i];
        }

        //  ret = xpci_writeCommon(msg, sizeof(MOD_saveConfigL));

        if(xpci_systemType == IMXPAD_S1400){
            ret = xpci_writeCommon_S1400(msg, sizeof(MOD_saveConfigL),modMask);
        }
        else{
            ret = xpci_writeCommon(msg, sizeof(MOD_saveConfigL));
        }
        free(msg);
        if (ret){
            printf("ERROR: %s() failed sending the request\n", __func__);
            return -1;
        }

        // Now wait for all ACKs on this loading
        ret = waitCommandReply(Mask, (char*)__func__, 15000);
    }//end module
    return ret;
}

// Store in the memory calibId the 7 values of the configuration register in the of 7 chips in the target 
// modules. 
//====================================================================================================
int   xpci_modSaveConfigG(unsigned modMask, unsigned calibId, unsigned reg, unsigned *values){
    int      i, module, mask;
    int      ret = 0;
    uint16_t *msg;
    unsigned Mask=0;
    unsigned selreg=0;


    int modules_nb = xpci_getModNb(modMask);
    int lastMod    = xpci_getLastMod(modMask);
    int FirstMod   = xpci_getFirstMod(modMask);

    // different imxpad systems consist in different number of modules
    //  xpadModuleType( &modules_nb , &start_module);

    if (modMask==0)
        return 0;

    for(module=FirstMod;module<lastMod;module++){
        mask = modMask & (1 << module);
        if(mask==0)
            continue;
        switch(xpci_systemType){
        case HUB:
        case IMXPAD_S70:
        case IMXPAD_S140:
        case IMXPAD_S340:
        case IMXPAD_S420:
        case IMXPAD_S540:
        case IMXPAD_S700:
        case IMXPAD_S1400:
            Mask = mask;
            selreg = reg; break;
        default:
            printf("ERROR: UNKNOWN detector type\n");
            return -1;
        }

        if (debugMsg) printf("Doing %s(0x%04x, 0x%04x, 0x%04x, ... )\n",  __func__,Mask, calibId, reg);

        // configure subchannel registers
        xpix_imxpadWriteSubchnlReg(Mask, 0, 1);

        msg = malloc(sizeof(MOD_saveConfigG));
        memcpy(msg,MOD_saveConfigG,sizeof(MOD_saveConfigG));
        msg[3]  = Mask;   // set the mask value
        msg[8]  = calibId;
        msg[9]  = selreg;      // target register
        for (i=0; i<7; i++)
            msg[10+i] = *(values+i);   // chips register values

        // ret = xpci_writeCommon(msg, sizeof(MOD_saveConfigG));
        if(xpci_systemType == IMXPAD_S1400){
            ret = xpci_writeCommon_S1400(msg, sizeof(MOD_saveConfigG),modMask);
        }
        else{
            ret = xpci_writeCommon(msg, sizeof(MOD_saveConfigG));
        }

        free(msg);
        if (ret){
            printf("ERROR: %s() failed sending the request\n", __func__);
            return -1;
        }

        // Now wait for all ACKs on this loading
        ret = waitCommandReply(Mask, (char*)__func__, 15000);
    }//modules
    return ret;
}
// Uploading of the calibration data contained in buffer "calibId" into the detector
//==================================================================================
int   xpci_modDetLoadConfig(unsigned modMask, unsigned calibId){
    int      ret = 0;
    uint16_t *msg;
    unsigned Mask=0;

    if (debugMsg) printf("Doing %s(0x%04x, 0x%04x, ... )\n",  __func__,modMask, calibId);
    
    if ( modMask==0)return 0;

    switch(xpci_systemType){
    case IMXPAD_S70:
    case IMXPAD_S140:
    case IMXPAD_S340:
    case IMXPAD_S420:
    case IMXPAD_S540:
    case IMXPAD_S700:
    case IMXPAD_S1400:
        Mask = modMask;
        break;
    default: printf("ERROR: UNKNOWN detector type\n");
        return -1;
    }

    // configure subchannel registers
    xpix_imxpadWriteSubchnlReg(Mask, 0, 1);

    msg = malloc(sizeof(MOD_detLoadConfig));
    memcpy(msg,MOD_detLoadConfig,sizeof(MOD_detLoadConfig));
    msg[3]  = Mask;   // set the mask value
    msg[8]  = calibId;
    
    // ret = xpci_writeCommon(msg, sizeof(MOD_detLoadConfig));
    if(xpci_systemType == IMXPAD_S1400){
        ret = xpci_writeCommon_S1400(msg, sizeof(MOD_detLoadConfig),modMask);
    }
    else{
        ret = xpci_writeCommon(msg, sizeof(MOD_detLoadConfig));
    }
    free(msg);
    if (ret){
        printf("ERROR: %s() failed sending the request\n", __func__);
        return -1;
    }

    // Now wait for ACK on this loading
    return ret = waitCommandReply(Mask, (char*)__func__, 15000);
}


// Enable detector's photons counting for the time in ms specified in the 32 bits "gateLength".
// During that process overflow bit is scanned and overflow counterss updated every 10 ms.
// gateLength should be given in ms.
// input: gateMode      0 internally generated
//                      1 external pulse
//        gateLength    (internal) length of the gate
//                      (external) number of pulses
//        timeUnit      ignored for external
//                      1- µsec    2- msec   3-sec
//        timeout       maximum time to wait in msec
//Example:
//1. Enable counters for two external pulses
//w[0] ? 1
//w[1] ? 2
//w[2] ? 0
//2. Enable counters for 500ms
//w[0] ? 0
//w[1] ? 500
//w[2] ? 2
// ATTENTION: Function does not aply to any of the IMXPAD systems
//===========================================================================================
int   xpci_modExpose(unsigned modMask, 
                     unsigned gateMode, unsigned gateLength, unsigned timeUnit,
                     int timeout){
    int      ret = 0;
    uint16_t *msg;
    unsigned Mask=modMask;

    //if (xpci_takeDetector()==-1){return -1;}
    // function aplies onlu to HUB and BACKPLANE systems
    if((xpci_systemType==HUB)||(xpci_systemType==BACKPLANE)){
        switch(xpci_systemType){
        case HUB:
            Mask = modMask;break;
        case BACKPLANE:
            Mask = modMask & 0x55;break;
        default:
            printf("ERROR: UNKNOWN detector type\n");
            return -1;
        }

        if (debugMsg) printf("%s(0x%04x, 0x%04x, 0x%04x)\n",  __func__,gateMode, gateLength, timeUnit);
        if ( modMask==0)
            return 0;
        msg = malloc(sizeof(MOD_expose));
        memcpy(msg,MOD_expose,sizeof(MOD_expose));
        msg[3]  = Mask;          // set the mask value
        msg[8]  = (uint16_t)gateMode;         // gate mode
        msg[9]  = (uint16_t)gateLength;    // gate length MSB
        msg[10] = (uint16_t)timeUnit;            // gate length LSB

        // ret = xpci_writeCommon(msg, sizeof(MOD_expose));
        if(xpci_systemType == IMXPAD_S1400){
            ret = xpci_writeCommon_S1400(msg, sizeof(MOD_expose),Mask);
        }
        else{
            ret = xpci_writeCommon(msg, sizeof(MOD_expose));
        }
        free(msg);
        if (ret){
            printf("ERROR: %s() failed sending the request\n", __func__);
            //xpci_freeDetector();
            return -1;
        }
        // Now wait for ACK on this loading

        //xpci_freeDetector();
        ret = waitCommandReply(Mask, (char*)__func__, timeout);
    }
    else{
        printf("ERROR: %s() function does not apply to any of the IMXPAD systems\n",  __func__);
        ret = -1;
    }


    return ret;
}
// Function to request exposition and get image 2B
// ATTENTION: Function does not aply to any of the IMXPAD systems
//=================================================
int   xpci_modImageGet2B(unsigned modMask, unsigned gateMode, unsigned gateLength, unsigned timeUnit){
    int      ret = 0;
    uint16_t *msg;
    unsigned Mask=0;

    // function aplies onlu to HUB and BACKPLANE systems
    if((xpci_systemType==HUB)||(xpci_systemType==BACKPLANE)){
        if (debugMsg) printf("%s(0x%04x, 0x%04x, 0x%04x)\n",  __func__,gateMode, gateLength, timeUnit);
        xpci_hubReset();
        if ( modMask==0)
            return 0;

        switch(xpci_systemType){
        case HUB:
            Mask = modMask;
        case BACKPLANE:
            Mask = (modMask & 0x55);
            break;
        default: printf("ERROR: UNKNOWN detector type\n");
            return -1;
        }

        msg = malloc(sizeof(MOD_getImg2B));
        memcpy(msg,MOD_getImg2B,sizeof(MOD_getImg2B));
        msg[3]  = Mask;          // set the mask value
        msg[8]  = (uint16_t)gateMode;         // gate mode
        msg[9]  = (uint16_t)gateLength;    // gate length
        msg[10] = (uint16_t)timeUnit;

        //  ret = xpci_writeCommon(msg, sizeof(MOD_getImg2B));
        if(xpci_systemType == IMXPAD_S1400){
            ret = xpci_writeCommon_S1400(msg, sizeof(MOD_getImg2B),Mask);
        }
        else{
            ret = xpci_writeCommon(msg, sizeof(MOD_getImg2B));
        }
        free(msg);
        if (ret){
            printf("ERROR: %s() failed sending the request\n", __func__);
            return -1;
        }
    }
    else{
        printf("ERROR: %s() function does not apply to any of the IMXPAD systems\n",  __func__);
        return -1;
    }
    return 0;
}

// ATTENTION: Function does not aply to any of the IMXPAD systems
//=================================================
int   xpci_modImageGet2B_XPAD32(unsigned modMask, unsigned gateMode, unsigned gateLength, unsigned timeUnit, unsigned nloop){
    int      ret = 0;
    uint16_t *msg;

    // function aplies only to HUB and BACKPLANE systems
    if((xpci_systemType==HUB)||(xpci_systemType==BACKPLANE)){
        if (debugMsg) printf("%s(0x%04x, 0x%04x, 0x%04x)\n",  __func__,gateMode, gateLength, timeUnit);
        xpci_hubReset();
        if ( modMask==0)
            return 0;
        msg = malloc(sizeof(MOD_getImg2B_XPAD32));
        memcpy(msg,MOD_getImg2B_XPAD32,sizeof(MOD_getImg2B_XPAD32));
        msg[3]  = modMask & 0x55;          // set the mask value
        msg[8]  = (uint16_t)gateMode;         // gate mode
        msg[9]  = (uint16_t)gateLength;    // gate length
        msg[10] = (uint16_t)timeUnit;
        msg[11] = (uint16_t)nloop;

        ret = xpci_writeCommon(msg, sizeof(MOD_getImg2B));
        free(msg);
        if (ret){
            printf("ERROR: %s() failed sending the request\n", __func__);
            return -1;

        }
    }
    else{
        printf("ERROR: %s() function does not apply to any of the IMXPAD systems\n",  __func__);
        return -1;
    }
    return 0;
}

// Function to request exposition and get image 4B
// ATTENTION: Function does not aply to any of the IMXPAD systems
//=================================================
int   xpci_modImageGet4B(unsigned modMask, unsigned gateMode, unsigned gateLength, unsigned timeUnit){
    int      ret = 0;
    uint16_t *msg;
    unsigned Mask=0;

    // function aplies only to HUB and BACKPLANE systems
    if((xpci_systemType==HUB)||(xpci_systemType==BACKPLANE)){
        if (debugMsg) printf("%s(0x%04x, 0x%04x, 0x%04x)\n",  __func__,gateMode, gateLength, timeUnit);
        xpci_hubReset();
        if ( modMask==0)
            return 0;

        switch(xpci_systemType){
        case HUB:
            Mask = modMask;
        case BACKPLANE:
            Mask = (modMask & 0x55);
            break;
        default: printf("ERROR: UNKNOWN detector type\n");
            return -1;
        }

        msg = malloc(sizeof(MOD_getImg4B));
        memcpy(msg,MOD_getImg4B,sizeof(MOD_getImg4B));
        msg[3]  = Mask;          // set the mask value
        msg[8]  = (uint16_t)gateMode;         // gate mode
        msg[9]  = (uint16_t)gateLength;    // gate length
        msg[10] = (uint16_t)timeUnit;

        ret = xpci_writeCommon(msg, sizeof(MOD_getImg4B));
        free(msg);
        if (ret){
            printf("ERROR: %s() failed sending the request\n", __func__);
            return -1;
        }
    }
    else{
        printf("ERROR: %s() function does not apply to any of the IMXPAD systems\n",  __func__);
        return -1;
    }
    return 0;
}

// Loads the same "value" in the DACL register in the module's chip and command the internal test pluse .

// ATTENTION: Function does not aply to any of the IMXPAD systems
//===========================================================================
int xpci_modPulseFlat(unsigned modMask, unsigned chipMask, unsigned valuedacl,unsigned amplPulse,unsigned NbHit){
    int      ret = 0;
    uint16_t *msg;
    unsigned Mask=0;

    // function aplies only to HUB and BACKPLANE systems
    if((xpci_systemType==HUB)||(xpci_systemType==BACKPLANE)){
        if (debugMsg) printf("Doing %s(0x%04x, 0x%04x, %d , %d)\n",  __func__,modMask,chipMask,valuedacl,amplPulse);
        xpci_hubReset();
        if ( modMask==0)
            return 0;

        switch(xpci_systemType){
        case HUB:
            Mask = modMask;
        case BACKPLANE:
            Mask = (modMask & 0x55);
            break;
        default: printf("ERROR: UNKNOWN detector type\n");
            return -1;
        }

        msg = malloc(sizeof(MOD_PulseFlat));
        memcpy(msg,MOD_PulseFlat,sizeof(MOD_PulseFlat));
        msg[3]  = modMask; // set the mask value
        msg[8] = Mask;
        msg[9] = valuedacl;
        msg[10] = amplPulse;
        msg[11] = NbHit;

        // ret = xpci_writeCommon(msg, sizeof(MOD_PulseFlat));
        if(xpci_systemType == IMXPAD_S1400){
            ret = xpci_writeCommon_S1400(msg, sizeof(MOD_PulseFlat),modMask);
        }
        else{
            ret = xpci_writeCommon(msg, sizeof(MOD_PulseFlat));
        }

        free(msg);
        if (ret){
            printf("ERROR: %s() failed sending the request\n", __func__);
            return -1;
        }
        //sleep(1);
        return ret = waitCommandReply(Mask, (char*)__func__, 9000);
    }
    else{
        printf("ERROR: %s() function does not apply to any of the IMXPAD systems\n",  __func__);
        return -1;
    }

}


// Loads the same "value" in the DACL register in the module's chip and command the internal test pluse .
// ATTENTION: Function does not aply to any of the IMXPAD systems
//===========================================================================
int xpci_modPulseConfig(unsigned modMask, unsigned chipMask, unsigned calibID,unsigned amplPulse,unsigned NbHit){
    int      ret = 0;
    uint16_t *msg;
    unsigned Mask=0;

    // function aplies only to HUB and BACKPLANE systems
    if((xpci_systemType==HUB)||(xpci_systemType==BACKPLANE)){
        if (debugMsg) printf("Doing %s(0x%04x, 0x%04x, %d , %d)\n",  __func__,modMask,chipMask,calibID,amplPulse);
        xpci_hubReset();
        if ( modMask==0)
            return 0;

        switch(xpci_systemType){
        case HUB:
            Mask = modMask;
        case BACKPLANE:
            Mask = (modMask & 0x55);
            break;
        default: printf("ERROR: UNKNOWN detector type\n");
            return -1;
        }

        msg = malloc(sizeof(MOD_PulseConfig));
        memcpy(msg,MOD_PulseConfig,sizeof(MOD_PulseConfig));
        msg[3]  = Mask; // set the mask value
        msg[8]  = chipMask;
        msg[9]  = calibID;
        msg[10] = amplPulse;
        msg[11] = NbHit;

        // ret = xpci_writeCommon(msg, sizeof(MOD_PulseConfig));
        if(xpci_systemType == IMXPAD_S1400){
            ret = xpci_writeCommon_S1400(msg, sizeof(MOD_PulseConfig),modMask);
        }
        else{
            ret = xpci_writeCommon(msg, sizeof(MOD_PulseConfig));
        }
        free(msg);
        if (ret){
            printf("ERROR: %s() failed sending the request\n", __func__);
            return -1;
        }
        printf("< %s() >\tWAIT 15 sec for execute the process \n", __func__);
        sleep(10); // ATTENTION REALLY NECESSARY

        return ret = waitCommandReply(Mask, (char*)__func__, 9000);
    }
    else{
        printf("ERROR: %s() function does not apply to any of the IMXPAD systems\n", __func__ );
        return -1;
    }
}



// Function to make a test loop test on the Hub to check that it is ready for use
// returns : 1 is OK true
//           O is KO false
//===============================================================================

// Test the hardware doing a loop test
// target: 0 = HUB 1 = PCIe
//======================================
static int isHardwareOK(enum DEVICES dev){
    int ret, i, error, chnl;
    uint16_t *rcvBuf;
    // should be a multiple of 16 bytes
    uint16_t   testMsg[] ={0xbb44, 0x3333, 0x0601, 0xaa55, 0x0300, 0xface, 0xf0f0, 0X0000};
    int        size      = sizeof(testMsg);

    if (xpci_systemType==BACKPLANE){
        printf("WARNING: No implementation of Hub and PCIe loop for BACKPLANE\n");
        return 1;
    }
    if (dev==HUB_DEV){
        xpci_setHubLoopMode(); if (debugMsg) printf("INFO: HUB Loopback mode set\n");
    }
    else if (dev==PCI_DEV){
        xpci_setPCILoopMode(); if (debugMsg) printf("INFO: PCIe Loopback mode set\n");
    }
    else
        return 1;

    rcvBuf = malloc(size);

    error=0;
    for (chnl=1; chnl>=0; chnl--){
        if      (dev==HUB_DEV) ret = xpci_write(chnl, testMsg, size);
        else if (dev==PCI_DEV) ret = xpci_writeTestPCI(chnl, testMsg, size);
        else    return 0;
        if (ret == -1) {
            xpci_getDmaStatus(chnl,1); // to print the error message
            printf("TEST ERROR: %s() can't send message on channel %d\n",  __func__,chnl);
            error++;
            goto end_isHardwareOK;
        }

        /* read back what as been received in the input fifo */
        /*===================================================*/
        ret = xpci_read(chnl, rcvBuf, size, 0);
        if (ret == -1) {
            xpci_getDmaStatus(chnl,1); // to print the error message
            printf("TEST ERROR: %s() when reading a test message on channel %d\n",  __func__,chnl);
            error++;
            goto end_isHardwareOK;
        }
        /* compare entry sent and recv data */
        /* Dump what we receive */
        if (debugMsg) printf("\t%s(%d): dumping received data ...",  __func__,chnl);
        for (i=0; i<size/sizeof(uint16_t); i++){
            if (debugMsg) printf("0x%04x>?>0x%04x  ", testMsg[i], rcvBuf[i]);
            if (testMsg[i]!=rcvBuf[i]) error++;
        }
        if (debugMsg) printf("\n");
    }
end_isHardwareOK:
    free(rcvBuf);
    if (dev==HUB_DEV){
        xpci_resetHubLoopMode();
    }
    else if (dev==PCI_DEV){
        xpci_resetPCILoopMode();
    }
    if (error) {
        printf("TEST HARDWARE ERROR: Check power supplies and fibers connections\n");
        return 0;
    }
    else
        return 1;
}
int xpci_isHubOK(){
    return isHardwareOK(HUB_DEV);
}

int xpci_isPCIeOK(){
    return isHardwareOK(PCI_DEV);

}


//===========================================================================
// top level function that executes a system dedicated function
//===========================================================================

// image sequence  function
// function embeds cppm and imxpad function
// The CPPM's implementation requires all arguments while imXPAD's implemetation ignores some
//===========================================================================
int   xpci_getImgSeq(enum IMG_TYPE type, int moduleMask, int nbChips, int nbImg, void **pBuff,
                     int gateMode_CPPM, int gateLength_CPPM, int timeUnit_CPPM, int firstTimeout_CPPM){
    int ret = 0;
//   if (acquisition_type == 0 )
        ret = xpci_getImgSeq_imxpad(type, moduleMask, nbChips, nbImg, pBuff);
//    else
//        ret = xpci_getImgSeq_SSD_imxpad(type, moduleMask, nbImg, gateMode_CPPM);

    return ret;
}

// Digital test function
//===========================================================================
int xpci_digitalTest(int modMask, int nbChips, uint16_t *pBuff, unsigned value, unsigned mode){

    if(xpci_modLoadAutoTest(modMask, value, mode)!=0)
        return -1;
    if(xpci_readOneImage(B2, modMask, nbChips, (void *)pBuff)!=0)
        return -1;
    
    return 0;
}

unsigned int xpci_getImageFormat(void)
{
		return img_Format_Acq;
}

//============================================================================== 
// Functions deticated for IMXPAD systems
//==============================================================================

// global exposure parameters
//===========================================================================
int xpci_modExposureParam( unsigned modMask,
                           unsigned Texp,
                           unsigned Twait,
                           unsigned Tinit,
                           unsigned Tshutter,
                           unsigned Tovf,
                           unsigned mode,
                           unsigned n, // n/p is a computed parameter
                           unsigned p,
                           unsigned nbImages,
                           unsigned BusyOutSel,
                           unsigned formatIMG, // 0 - 16 bits, 1 - 32 bits
                           unsigned postProc, // post processing
                           unsigned GP1,
                           unsigned AcqMode, // Aquisition type
                           unsigned StakingOrBunchMode, // staking & sihgle bunch 
                           unsigned GP4  // do not use
                           ){
    int      ret = 0;
    uint16_t *msg;
    unsigned Tshut_real = 0;
    unsigned Twait_real = Twait;
    unsigned imageFormat=0;
    unsigned Aqc_mod_param;

    acquisition_type = AcqMode;
    Aqc_mod_param = AcqMode;
    usleep(5000);

	if(modMask == 0){
		printf("Lib_xpci => ERROR : Bad value modMask\n");
		return -2;
	}
	if(nbImages < 0 || nbImages > 60000){
		printf("Lib_xpci => ERROR : Bad value nImage\n");
		return -2;
	}

	if(AcqMode == 0){
		if(Texp > 16*Tovf)
			imageFormat =  1;
		 else
			imageFormat =  0;
	}
	else if(AcqMode == 1)
		imageFormat = 0;
	else if(AcqMode == 2)
		imageFormat = 0;
	else if(AcqMode == 3){
		Aqc_mod_param = 3;
		imageFormat = 0;
	}
	else if(AcqMode == 4){
		Aqc_mod_param = 3;
		imageFormat = 1;
	}
	else if(AcqMode == 5){
		Aqc_mod_param = 4;
		imageFormat = 0;
	}
	else if(AcqMode == 6){
		Aqc_mod_param = 4;
		imageFormat = 1;
	}
	else if(AcqMode == 7){
		if(Texp > 16*Tovf)
			imageFormat =  1;
		 else
			imageFormat =  0;
	}
	else{
		printf("Lib_xpci => ERROR : Bad value AqcMode\n");
		return -2; 
	}
		
	
	
	img_Format_Acq = imageFormat;
	//printf( " %s() ==> AcqMode = %d Aqc_mod_param = %d imageFormat = %d img_Format_Acq = %d \n",__func__ ,AcqMode,Aqc_mod_param,imageFormat,img_Format_Acq);
	
    if(Tshutter>Texp)
        Tshut_real = 0;
    else
        Tshut_real = Texp-Tshutter;

    if(xpci_systemType == IMXPAD_S1400 || xpci_systemType == IMXPAD_S700){
        if(Twait_real>=20) Twait_real = Twait - 20;
    }

    if (debugMsg) printf("Doing %s \n",  __func__);
    if ( modMask==0)
        return 0;

    imxpad_postProc = postProc;

    // configure subchannel registers
    xpix_imxpadWriteSubchnlReg(modMask, 0, 1);

    msg = malloc(sizeof(MOD_exposureParam));
    memcpy(msg, MOD_exposureParam, sizeof(MOD_exposureParam));
    msg[3]  = (uint16_t)modMask;
    msg[8]  = Texp >> 16;    // higher 16 bits
    msg[9]  = Texp & 0xffff; // lower 16 bits
    msg[10] = Twait_real >>16;
    msg[11] = Twait_real & 0xffff;;
    msg[12] = Tinit >> 16;
    msg[13] = Tinit & 0xffff;
    msg[14] = Tshut_real >> 16;
    msg[15] = Tshut_real & 0xffff;
    msg[16] = Tovf >> 16;
    msg[17] = Tovf & 0xffff;
    msg[18] = mode;
    msg[19] = n;
    msg[20] = p;
    msg[21] = nbImages;
    msg[22] = BusyOutSel;
    msg[23] = imageFormat;
    msg[24] = postProc;
    msg[25] = GP1;
    msg[26] = Aqc_mod_param;
    msg[27] = StakingOrBunchMode >> 16;       //GP3;
    msg[28] = StakingOrBunchMode & 0xffff;    //GP4;
    //ret = xpci_writeCommon(msg, sizeof(MOD_exposureParam));
    if(xpci_systemType == IMXPAD_S1400){
        ret = xpci_writeCommon_S1400(msg, sizeof(MOD_exposureParam),modMask);
    }
    else{
        ret = xpci_writeCommon(msg, sizeof(MOD_exposureParam));
    }
    free(msg);
    if (ret){
        printf("ERROR: %s() failed sending the request\n", __func__);
        //xpci_freeDetector();
        return -1;
    }
    // Now wait for ACK on this loading
    return waitCommandReply(modMask, (char*)__func__, 15000);
}


int xpci_modExposureParam_internal( unsigned modMask,
                           unsigned Texp,
                           unsigned Twait,
                           unsigned Tinit,
                           unsigned Tshutter,
                           unsigned Tovf,
                           unsigned mode,
                           unsigned n, // n/p is a computed parameter
                           unsigned p,
                           unsigned nbImages,
                           unsigned BusyOutSel,
                           unsigned formatIMG, // 0 - 16 bits, 1 - 32 bits
                           unsigned postProc, // post processing
                           unsigned GP1,
                           unsigned AcqMode, // Aquisition type
                           unsigned StakingOrBunchMode, // staking & sihgle bunch 
                           unsigned GP4  // do not use
                           ){
    int      ret = 0;
    uint16_t *msg;
    unsigned Tshut_real = 0;
    unsigned Twait_real = Twait;
    unsigned imageFormat=0;
    unsigned Aqc_mod_param;

    acquisition_type = AcqMode;
    Aqc_mod_param = AcqMode;
    usleep(5000);
 /*   
    if(AcqMode == 1 || AcqMode == 2)
		imageFormat = 0;
	else
		imageFormat = formatIMG;
*/

	if(AcqMode == 0){
		 imageFormat =  formatIMG;
	}
	else if(AcqMode == 1)
		imageFormat = 0;
	else if(AcqMode == 2)
		imageFormat = 0;
	else if(AcqMode == 3){
		Aqc_mod_param = 3;
		imageFormat = 0;
	}
	else if(AcqMode == 4){
		Aqc_mod_param = 3;
		imageFormat = 1;
	}
	else if(AcqMode == 5){
		Aqc_mod_param = 4;
		imageFormat = 0;
	}
	else if(AcqMode == 6){
		Aqc_mod_param = 4;
		imageFormat = 1;
	}
	
	
	img_Format_Acq = imageFormat;
	//printf( " %s() ==> AcqMode = %d Aqc_mod_param = %d imageFormat = %d img_Format_Acq = %d \n",__func__ ,AcqMode,Aqc_mod_param,imageFormat,img_Format_Acq);
	
    if(Tshutter>Texp)
        Tshut_real = 0;
    else
        Tshut_real = Texp-Tshutter;

    if(xpci_systemType == IMXPAD_S1400 || xpci_systemType == IMXPAD_S700){
        if(Twait_real>=20) Twait_real = Twait - 20;
    }

    if (debugMsg) printf("Doing %s \n",  __func__);
    if ( modMask==0)
        return 0;

    imxpad_postProc = postProc;

    // configure subchannel registers
    xpix_imxpadWriteSubchnlReg(modMask, 0, 1);

    msg = malloc(sizeof(MOD_exposureParam));
    memcpy(msg, MOD_exposureParam, sizeof(MOD_exposureParam));
    msg[3]  = (uint16_t)modMask;
    msg[8]  = Texp >> 16;    // higher 16 bits
    msg[9]  = Texp & 0xffff; // lower 16 bits
    msg[10] = Twait_real >>16;
    msg[11] = Twait_real & 0xffff;;
    msg[12] = Tinit >> 16;
    msg[13] = Tinit & 0xffff;
    msg[14] = Tshut_real >> 16;
    msg[15] = Tshut_real & 0xffff;
    msg[16] = Tovf >> 16;
    msg[17] = Tovf & 0xffff;
    msg[18] = mode;
    msg[19] = n;
    msg[20] = p;
    msg[21] = nbImages;
    msg[22] = BusyOutSel;
    msg[23] = imageFormat;
    msg[24] = postProc;
    msg[25] = GP1;
    msg[26] = Aqc_mod_param;
    msg[27] = StakingOrBunchMode >> 16;       //GP3;
    msg[28] = StakingOrBunchMode & 0xffff;    //GP4;
    //ret = xpci_writeCommon(msg, sizeof(MOD_exposureParam));
    if(xpci_systemType == IMXPAD_S1400){
        ret = xpci_writeCommon_S1400(msg, sizeof(MOD_exposureParam),modMask);
    }
    else{
        ret = xpci_writeCommon(msg, sizeof(MOD_exposureParam));
    }
    free(msg);
    if (ret){
        printf("ERROR: %s() failed sending the request\n", __func__);
        //xpci_freeDetector();
        return -1;
    }
    // Now wait for ACK on this loading
    return waitCommandReply(modMask, (char*)__func__, 15000);
}




// send pulses to the enabled pixels from the internal pulser block
//===========================================================================
int xpci_pulserImxpad(unsigned modMask, unsigned Pnum, unsigned Pmask){
    int	   ret= 0;
    uint16_t *msg;

    if (debugMsg) printf("Doing %s(0x%04x)\n",  __func__,modMask);
    if ( modMask==0)
        return 0;
    msg = malloc(sizeof(MOD_PulserImxpad));
    memcpy(msg, MOD_PulserImxpad,sizeof(MOD_PulserImxpad));
    msg[3]  = (uint16_t)modMask; // set the mask value
    msg[8]  = (uint16_t)Pnum;
    msg[9]  = (uint16_t)Pmask;

    if ( modMask==0)
        return 0;

    // configure subchannel registers
    xpix_imxpadWriteSubchnlReg(modMask, 0, 1);

    // ret = xpci_writeCommon(msg, sizeof(MOD_PulserImxpad));
    if(xpci_systemType == IMXPAD_S1400){
        ret = xpci_writeCommon_S1400(msg, sizeof(MOD_PulserImxpad),modMask);
    }
    else{
        ret = xpci_writeCommon(msg, sizeof(MOD_PulserImxpad));
    }

    free(msg);
    if (ret){
        printf("ERROR: %s() failed sending the request\n" ,__func__);
        return -1;
    }

    return waitCommandReply(modMask, (char*)__func__, 500000);
}


// function to expose and read detector N times
// function is compatible only with IMXPAD systems
//===========================================================================
int xpci_getImgSeq_imxpad(enum IMG_TYPE type, int modMask, int nChips, int nImg, void **pBuff){
    int             ret = 0;
    int             i = 0;
    uint16_t        *msg;
    int             imgSize = 0;
    int             modNb = xpci_getModNb(modMask);
    int             lastMod = xpci_getLastMod(modMask);
    uint16_t        **pRawBuff;
    
    /*
    printf("type = %d",type);
    printf("type = %d",modMask);
    printf("type = %d",nChips);
    printf("type = %d",nImg);
*/
    printf("%s ---> Starting acquisition...  ", __func__);
    // Variables for Async Reading
    int              fd;
    unsigned int     *imageNumber;
    unsigned int     numPixels;
    unsigned short   *image16;
    unsigned int     *image32;

    img_gotImages = 0;

    // check if any of the modules enabled
    if ( modMask==0)
        return 0;
	xpci_clearAbortProcess();
	xpci_clearResetProcess();
    // check if detector is available
    if (xpci_modGlobalAskReady(modMask)!=0){
        printf("ERROR: %s() ---> failed sending AskReady\n", __func__);
        return -1;
    }

    // determine size of the image
    if(type==B2){
        imgSize = 120*566*lastMod*sizeof(uint16_t);
    }
    else{
        imgSize = 120*1126*lastMod*sizeof(uint16_t);
    }

    numPixels = 120*560*lastMod;

    // configure subchannel registers
    if(type==B2)
        xpix_imxpadWriteSubchnlReg(modMask, 1, nImg);
    else
        xpix_imxpadWriteSubchnlReg(modMask, 2, nImg);

    pRawBuff = malloc(nImg*sizeof(uint16_t *));
    for(i=0; i<nImg; i++){
        pRawBuff[i] = malloc(imgSize);
        // memset(pRawBuff[i],0,imgSize);
    }

    // initialize image structure
    if(xpci_readImageInit(type, modMask, nChips)==-1){
        printf("ERROR: %s() ---> image acquisition init FAILED\n", __func__);
        return -1;
    }

    // disable timeout hardware (wait forever for the data to arrive)
    xpci_setHardTimeout(HWTIMEOUT_DSBL);

    // send expose message (do not wait for a reply)
    msg = malloc(sizeof(MOD_expose));
    memcpy(msg,MOD_expose,sizeof(MOD_expose));
    msg[3]  =  (uint16_t)modMask;          // set the mask value
    //  ret = xpci_writeCommon(msg, sizeof(MOD_expose));
    if(xpci_systemType == IMXPAD_S1400){
        ret = xpci_writeCommon_S1400(msg, sizeof(MOD_expose),modMask);
    }
    else{
        ret = xpci_writeCommon(msg, sizeof(MOD_expose));
    }
    free(msg);
    if (ret){
        printf("ERROR: %s() ---> failed sending the request\n", __func__);
        return -1;
    }
    flag_startExpose = 1;
    if(pBuff == NULL)
    {
        //**************** Start of async variable set ****************
        //**************** imageNumber ****************                 //Number of the last image acquired
        // Create a new memory object
        fd = shm_open( "/imageNumber", O_RDWR | O_CREAT, 0777 );
        if( fd == -1 ) {
            fprintf( stderr, "ERROR %s() ---> Open failed [imageNumber]:%s\n",__func__,
                     strerror( errno ) );
            return -1;
        }

        // Set the memory object's size
        if( ftruncate( fd, sizeof( *imageNumber ) ) == -1 ) {
            fprintf( stderr, "ERROR %s() ---> ftruncate [imageNumber]:%s\n",__func__,
                     strerror( errno ) );
            return -1;
        }

        // Map the memory object
        imageNumber = (unsigned int *) mmap( NULL, sizeof( *imageNumber ),
                                             PROT_WRITE,
                                             MAP_SHARED, fd, 0 );
        if( imageNumber == MAP_FAILED ) {
            fprintf( stderr, "ERROR %s() ---> imageNumber mmap failed [imageNumber]:%s\n",__func__,
                     strerror( errno ) );
            return -1;
        }

        close(fd);

        //**************** images ****************                      //Shared memory where images will be stored
        // Create a new memory object
        fd = shm_open( "/images", O_RDWR | O_CREAT, 0777 );
        if( fd == -1 ) {
            fprintf( stderr, "ERROR: %s() ---> Open failed [images]:%s\n",__func__,
                     strerror( errno ) );
            return -1;
        }




        // Map the memory object
        if(type==B2){
            // Set the memory object's size
            if( ftruncate( fd, sizeof( *image16 )*nImg*numPixels ) == -1 ) {
                fprintf( stderr, "ERROR: %s() ---> ftruncate [images]:%s\n",__func__,
                         strerror( errno ) );
                return -1;
            }
            image16 = (unsigned short *) mmap( NULL, sizeof( *image16 )*nImg*numPixels,
                                               PROT_WRITE,
                                               MAP_SHARED, fd, 0 );
            if( image16 == MAP_FAILED ) {
                fprintf( stderr, "ERROR: %s() ---> image mmap failed [images]:%s\n",__func__,
                         strerror( errno ) );
                return -1;
            }
        }
        else{
            // Set the memory object's size
            if( ftruncate( fd, sizeof( *image32 )*nImg*numPixels ) == -1 ) {
                fprintf( stderr, "ERROR: %s() ---> ftruncate [images]:%s\n",__func__,
                         strerror( errno ) );
                return -1;
            }
            image32 = (unsigned int *) mmap( NULL, sizeof( *image32 )*nImg*numPixels,
                                             PROT_WRITE,
                                             MAP_SHARED, fd, 0 );
            if( image32 == MAP_FAILED ) {
                fprintf( stderr, "ERROR: %s() ---> image mmap failed [images]:%s\n",__func__,
                         strerror( errno ) );
                return -1;
            }
        }

        close(fd);
    }
    //**************** End of async variable set ****************

        if(xpci_getAbortProcess()){
            printf("%s() ---> Last Acquired Image = %d\n",__func__, i);
            return 1;
        }
    // Reading images and copying to shared memory
    printf("\n");
    for (i=0; i<nImg; i++){
        if(xpci_readImgBuff( *(pRawBuff+i), 0)==-1 ){
            printf("ERROR: %s() ---> image %d reading FAILED\n", __func__, i);
            ret =-1;
        }  
        
        if(pBuff != NULL)
        {
            // extract and organize data from the raw image(s)
            if(type==B2){
               // imxpad_raw2data_16bits_v2(modMask, pRawBuff[i], (uint16_t **)pBuff,i);
                imxpad_raw2data_16bits(modMask, pRawBuff[i], (uint16_t *)pBuff[i]);
                if(i > 5 ) // delete only the images n - 5 because the data tranfert
                    free(pRawBuff[i - 5]); //__fred__
            }
            else{
                imxpad_raw2data_32bits(modMask, pRawBuff[i], (uint32_t *)pBuff[i]);
                free(pRawBuff[i]);
            }
        }
        else
        {
            // extract and organize data from the raw image(s)
            if(type==B2){
                uint16_t table[numPixels];
                imxpad_raw2data_16bits(modMask, pRawBuff[i], table);
                free(pRawBuff[i]);
                for(int j=0; j<numPixels; j++)
                    image16[i*numPixels+j] = table[j];
            }
            else{
                uint32_t table[numPixels];
                imxpad_raw2data_32bits(modMask, pRawBuff[i], table);
                free(pRawBuff[i]);
                for(int j=0; j<numPixels; j++)
                    image32[i*numPixels+j] = table[j];
            }
            imageNumber[0] = i+1;
        }
        img_gotImages++;
        if(xpci_getAbortProcess()){
            printf("%s() ---> Last Acquired Image = %d\n",__func__, i);
            break;
        }
        if(xpci_getResetProcess()){
            printf("%s() ---> Last Acquired Image = %d\n",__func__, i);
            break;
        }
    } // end of loop for nImg

    // restore short hw timeout
    xpci_setHardTimeout(HWTIMEOUT_1SEC);
    xpci_getImageClose();
    
    if(pBuff == NULL){
        if(type==B2)
            munmap(image16,sizeof( *image16 )*nImg*numPixels);
        else
            munmap(image32,sizeof( *image32 )*nImg*numPixels);

        munmap(imageNumber,sizeof( *imageNumber ));
    }// pBuff == NULL
    else if(type==B2){//__fred__
        if ( nImg > 5 ){
            for(i=nImg-5; i<nImg; i++)
                free(pRawBuff[i]);
        }
        else{
            for(i=0; i<nImg; i++)
                free(pRawBuff[i]);
        }
        free(pRawBuff);
    } // type == B2
    
    if(!xpci_getAbortProcess()){
        xpci_modAbortExposure();
        xpci_clearAbortProcess();
    }
    xpci_AbortCleanDetector(modMask);
    printf("%s ---> Acquisition finished\n", __func__);
    flag_startExpose = 0;
    xpci_clearResetProcess();
    return ret;
}

unsigned int get_flagStartExpose (void)
{
	return flag_startExpose;
}

int xpci_getImgSeq_SSD_imxpad(enum IMG_TYPE type, int modMask, int nImg, int burstNumber){
    int             ret = 0;
    int             i = 0,j = 0;
    uint16_t        *msg;
    int             imgSize = 0,outSize;
    int             modNb = xpci_getLastMod(modMask);
    int             lastMod = xpci_getLastMod(modMask);
    //    uint16_t        *pRawBuff_0,*pRawBuff_1;
    char            fname_0[100];
    pthread_t       WriteFileThread;
    // Variables for Async Reading
    int              fd;
    unsigned int     *imageNumber;
    char str[200];    
    unsigned par[4];
    int ii,jj;    
    int maxImgBuff = (int) (nImg * 0.025) + 10;
    
    
    xpci_clearAbortProcess();
    xpci_clearResetProcess();
    xpci_clearNumberLastAcquiredAsyncImage();

    // determine size of the image
    if(type==B2){
        imgSize = 120*566*lastMod*sizeof(uint16_t);
    }
    else{
        imgSize = 120*1126*lastMod*sizeof(uint16_t);
    }

    par[0] = imgSize;
    par[1] = burstNumber;
    par[2] = nImg;
    par[3] = maxImgBuff;
    
        // check if detector is available
    if (nImg>65000){
        printf("ERROR: %s() ---> failed numbers of images.\n", __func__);
        flag_startExpose = -1;
        return -1;
    }
    
    printf("%s ---> Starting acquisition...  maxImgBuff = %d ", __func__,maxImgBuff);
    // check if any of the modules enabled
    if ( modMask==0)
        return 0;

    // check if detector is available
    if (xpci_modGlobalAskReady(modMask)!=0){
        printf("ERROR: %s() ---> failed sending AskReady.\n", __func__);
        flag_startExpose = -1;
        return -1;
    }

    sprintf(str,"rm /opt/imXPAD/tmp/burst_%d*.bin",burstNumber);
    system(str);
    
    // configure subchannel registers
    if(type==B2)
        xpix_imxpadWriteSubchnlReg(modMask, 1, nImg);
    else
        xpix_imxpadWriteSubchnlReg(modMask, 2, nImg);
   
	pRawBuff_ssd = malloc(IMG_SSD * sizeof(uint16_t*));
	if(pRawBuff_ssd == NULL ){
        printf("ERROR: %s ---> Can not create data buffer.\n",__func__);
        flag_startExpose = -1;
        return -1;
    }
	for(i=0;i<maxImgBuff;i++){
		pRawBuff_ssd[i] = malloc(imgSize);
		if(pRawBuff_ssd[i] == NULL ){
			printf("ERROR: %s ---> Can not create data buffer.\n",__func__);
			flag_startExpose = -1;
			return -1;
		}
	}
  
    write_pRawBuff_ssd = 0; // init number of image
    read_pRawBuff_ssd  = 0; // init number of image
    
    if (ret = pthread_create(&WriteFileThread,
                             NULL,
                             &xpci_writeRawDataToFile,
                             par) !=0){
        printf("ERROR: %s ---> Thread creation FAILED.\n", __func__);
        flag_startExpose = -1;
        return -1;
    }
    else {
        printf("OK: thread write raw data creation is success.\n");
        pthread_yield(); // let opportunity for thread real start
    }

    // initialize image structure
    if(xpci_readImageInit(type, modMask, 7)==-1){
        printf("ERROR %s() ---> Image acquisition init FAILED.\n", __func__);
        flag_startExpose = -1;
        return -1;
    }

    // disable timeout hardware (wait forever for the data to arrive)
    xpci_setHardTimeout(HWTIMEOUT_DSBL);

    // send expose message (do not wait for a reply)
    msg = malloc(sizeof(MOD_expose));
    memcpy(msg,MOD_expose,sizeof(MOD_expose));
    msg[3]  =  (uint16_t)modMask;          // set the mask value
    if(xpci_systemType == IMXPAD_S1400){
        ret = xpci_writeCommon_S1400(msg, sizeof(MOD_expose),modMask);
    }
    else{
        ret = xpci_writeCommon(msg, sizeof(MOD_expose));
    }
    free(msg);
    if (ret){
        printf("ERROR: %s() ---> Sending the request FAILED\n", __func__);
        flag_startExpose = -1;
        return -1;
    }
    flag_startExpose = 1;
    
    xpci_timerStart(3);
    for (i=0; i<nImg; i++){
		while(read_pRawBuff_ssd > (write_pRawBuff_ssd + maxImgBuff - 1)); // wait data write to file
		if(xpci_readImgBuff( pRawBuff_ssd[i%maxImgBuff], 0)==-1 ){
			printf("ERROR %s(): ---> image %d reading FAILED.\n", __func__, i);
			ret =-1;
		}
		read_pRawBuff_ssd++;    
		//printf("read_pRawBuff_ssd = %d\n",read_pRawBuff_ssd); 
        if(xpci_getAbortProcess()){
			read_pRawBuff_ssd++; 
            printf("%s() ---> Last Acquired Image = %d.\n",__func__, i);
            break;
        }
        //printf("read_pRawBuff_ssd = %d\n",read_pRawBuff_ssd); 
        if(xpci_getResetProcess()){
			read_pRawBuff_ssd++; 
            printf("%s() ---> Last Acquired Image = %d.\n",__func__, i);
            break;
        }        
    }
    xpci_timerStop(3);
    printf("%s() ---> Waiting thread.\n", __func__);
    pthread_join (WriteFileThread,NULL);

	for(i=0;i<maxImgBuff;i++)
		free(pRawBuff_ssd[i]);
	free(pRawBuff_ssd);

    // restore short hw timeout
    xpci_setHardTimeout(HWTIMEOUT_1SEC);
    xpci_getImageClose();
    if(!xpci_getAbortProcess()){
        xpci_modAbortExposure();
        xpci_clearAbortProcess();
    }
    xpci_AbortCleanDetector(modMask);
    xpci_clearResetProcess();
    printf("%s() ---> Expose finished.\n", __func__);
    flag_startExpose = 0;
    return ret;
}


// extended  waitCommandReply function that can return detector message to the user
//===========================================================================
int waitCommandReplyExtended(unsigned modMask, char *userFunc, int timeout, unsigned *detRet){
    struct ModuleReplyBuffer rcvBuf[10];
    int i, chnl, nbMod;
    int ret = 0;
    uint16_t *rptr;
    
    memset(rcvBuf,0,sizeof(rcvBuf));
    for ( chnl=0; chnl<=1; chnl++){
        if (!isChannelUsed(chnl, modMask))
            continue;
        nbMod = nbModOnChannel(chnl, modMask);
        ret = xpci_read(chnl, (uint16_t*)&rcvBuf[chnl].data, MOD_REPLY_SIZE*nbMod, timeout);
        if (ret) {
            printf("ERROR: %s()/%s()  failed reading the reply on %d\n", __func__,userFunc,chnl);
            return -1;
        }
        rptr = (uint16_t*)(rcvBuf[chnl].data);
        for (i=0; i<((MOD_REPLY_SIZE*nbMod)/sizeof(uint16_t)); i++){
            *detRet = rptr[i];
            detRet++;
        }

    }//for chnl
    return 0;
}

// function to read a selected global register from the modules
//===========================================================================

int xpci_modReadConfigG(unsigned modMask, unsigned chipMask, unsigned reg, unsigned *detRet){
    int	ret, i, j= 0;
    uint16_t *msg;
    int lastMod  = xpci_getLastMod(modMask);
    unsigned detData[16*lastMod];
    unsigned mod = 0;

    if (debugMsg) printf("Doing %s(0x%04x)\n",  __func__,modMask);
    if ( modMask==0)
        return 0;
    msg = malloc(sizeof(MOD_readConfigG));
    memcpy(msg,MOD_readConfigG,sizeof(MOD_readConfigG));
    msg[3]  = (uint16_t)modMask; // set the mask value
    msg[8] = (uint16_t)chipMask;
    msg[9] = (uint16_t)reg;

    // configure subchannel registers
    xpix_imxpadWriteSubchnlReg(modMask, 0, 1);
    //ret = xpci_writeCommon(msg, sizeof(MOD_readConfigG));
    if(xpci_systemType == IMXPAD_S1400){
        ret = xpci_writeCommon_S1400(msg, sizeof(MOD_readConfigG),modMask);
    }
    else{
        ret = xpci_writeCommon(msg, sizeof(MOD_readConfigG));
    }
    free(msg);
    if (ret){
        printf("ERROR: %s() failed sending the request\n" ,__func__);
        return -1;

    }
    ret = waitCommandReplyExtended(modMask, (char*)__func__, 10000, &detData[0]);
    for(i=0; i<lastMod; i++){
        mod = detData[i*16+1]-1;
        if((modMask & (1 << mod))==0) continue;
        for(j=0; j<7; j++)
            detRet[mod*7+j]=detData[i*16+5+j];
    }
    return ret;
}


int xpci_modReadTempSensor(unsigned modMask, float *detRet){
    int	ret, i, j= 0;
    uint16_t *msg;
    int lastMod  = xpci_getLastMod(modMask);
    unsigned detData[16*lastMod];
    unsigned mod = 0;

    if (debugMsg) printf("Doing %s(0x%04x)\n",  __func__,modMask);
    if ( modMask==0)
        return 0;
    msg = malloc(sizeof(MOD_TempSensor));
    memcpy(msg,MOD_TempSensor,sizeof(MOD_TempSensor));
    msg[3]  = (uint16_t)modMask; // set the mask value

    // configure subchannel registers
    xpix_imxpadWriteSubchnlReg(modMask, 0, 1);
    //ret = xpci_writeCommon(msg, sizeof(MOD_readConfigG));
    if(xpci_systemType == IMXPAD_S1400){
        ret = xpci_writeCommon_S1400(msg, sizeof(MOD_TempSensor),modMask);
    }
    else{
        ret = xpci_writeCommon(msg, sizeof(MOD_TempSensor));
    }
    free(msg);
    if (ret){
        printf("ERROR: %s() failed sending the request\n" ,__func__);
        return -1;

    }
    ret = waitCommandReplyExtended(modMask, (char*)__func__, 10000, &detData[0]);
    for(i=0; i<lastMod; i++){
        mod = detData[i*16+1]-1;
        if((modMask & (1 << mod))==0) continue;
        for(j=0; j<7; j++)
            detRet[mod*7+j]=detData[i*16+4+j]*2.5-465;
            
    }
    return ret;
}




// function to send waiting times between exposures
//===========================================================================
int xpci_modSendExpWaitTimes(unsigned modMask, unsigned startAddr, unsigned endAddr, unsigned *waitTimes){
    int ret = 0;
    int i = 0;
    uint16_t *msg;
    unsigned nbAddr = endAddr - startAddr;

    if (debugMsg) printf("Doing %s(0x%04x)\n",  __func__,modMask);
    if ( modMask==0)
        return 0;
    msg = malloc(sizeof(MOD_saveWaitExpTimes));
    memcpy(msg,MOD_saveWaitExpTimes,sizeof(MOD_saveWaitExpTimes));

    msg[3]  = (uint16_t)modMask; // set the mask value
    msg[8] = (uint16_t)startAddr;
    msg[9] = (uint16_t)endAddr;
    for(i=0;i<nbAddr;i++){
        msg[10+i*2] = *(waitTimes+i)>>16;
        msg[10+i*2+1] = *(waitTimes+i) & 0xffff;
    }
    if ( modMask==0)
        return 0;

    // configure subchannel registers
    xpix_imxpadWriteSubchnlReg(modMask, 0, 1);

    // ret = xpci_writeCommon(msg, sizeof(MOD_saveWaitExpTimes));
    if(xpci_systemType == IMXPAD_S1400){
        ret = xpci_writeCommon_S1400(msg, sizeof(MOD_saveWaitExpTimes),modMask);
    }
    else{
        ret = xpci_writeCommon(msg, sizeof(MOD_saveWaitExpTimes));
    }
    free(msg);
    if (ret){
        printf("ERROR: %s() failed sending the request\n" ,__func__);
        return -1;
    }

    ret = waitCommandReply(modMask, (char*)__func__, 10000000);
    return ret;
}

// function to abort exposure
//===========================================================================
int xpci_modAbortExposure(){
    int ret = 0;
    int status = 0;
    int chnl;

    if (debugMsg) printf("Doing %s\n",  __func__);
    xpci_setAbortProcess();
	ret = xpci_writeSplitted_resetTX(1, MOD_imxpadAbortExp, sizeof(MOD_imxpadAbortExp));
	if (ret) status = ret;
	ret = xpci_writeSplitted_resetTX(0, MOD_imxpadAbortExp, sizeof(MOD_imxpadAbortExp));
	
//	ret = xpci_writeSplitted_resetTX(3, MOD_imxpadAbortExp, sizeof(MOD_imxpadAbortExp));
	if (ret) status = ret;
    if (status){
        printf("ERROR: %s() failed sending the request\n", __func__);
        return -1;
    }
    
    return status;
}



// function to do stacking of the images
// function requires specific firmware
//===========================================================================
int xpci_sendIPI(unsigned modMask, unsigned nloop){
    int	ret= 0;
    uint16_t *msg;

    if (debugMsg) printf("Doing %s(0x%04x)\n",  __func__,modMask);
    if ( modMask==0)
        return 0;

    // configure subchannel registers
    xpix_imxpadWriteSubchnlReg(modMask, 0, 1);

    msg = malloc(sizeof(MOD_sendIPI));
    memcpy(msg,MOD_sendIPI,sizeof(MOD_sendIPI));
    msg[3]  = (uint16_t)modMask; // set the mask value
    msg[8]  = (uint16_t)nloop;

    if ( modMask==0)
        return 0;

    // ret = xpci_writeCommon(msg, sizeof(MOD_sendIPI));
    if(xpci_systemType == IMXPAD_S1400){
        ret = xpci_writeCommon_S1400(msg, sizeof(MOD_sendIPI),modMask);
    }
    else{
        ret = xpci_writeCommon(msg, sizeof(MOD_sendIPI));
    }
    free(msg);
    if (ret){
        printf("ERROR: %s() failed sending the request\n" ,__func__);
        return -1;
    }

    ret = waitCommandReply(modMask, (char*)__func__, 10000000);

    return ret;
}

// function to configure stacking processing in the detector
// function requires specific firmware
//===========================================================================
int xpci_sendIPIParam(unsigned modMask, unsigned IPI_ena, unsigned IPI_nr){
    int	ret= 0;
    uint16_t *msg;

    if (debugMsg) printf("Doing %s(0x%04x)\n",  __func__,modMask);
    if ( modMask==0)
        return 0;

    // configure subchannel registers
    xpix_imxpadWriteSubchnlReg(modMask, 0, 1);

    msg = malloc(sizeof(MOD_sendIPIParam));
    memcpy(msg, MOD_sendIPIParam, sizeof(MOD_sendIPIParam));
    msg[3]  = (uint16_t)modMask; // set the mask value
    msg[8]  = (uint16_t)IPI_ena;
    msg[9]  = (uint16_t)IPI_nr;

    if ( modMask==0)
        return 0;

    // ret = xpci_writeCommon(msg, sizeof(MOD_IPI_PARAM));
    if(xpci_systemType == IMXPAD_S1400){
        ret = xpci_writeCommon_S1400(msg, sizeof(MOD_IPI_PARAM),modMask);
    }
    else{
        ret = xpci_writeCommon(msg, sizeof(MOD_IPI_PARAM));
    }
    free(msg);
    if (ret){
        printf("ERROR: %s() failed sending the request\n" ,__func__);
        return -1;
    }

    ret = waitCommandReply(modMask, (char*)__func__, 1000000);

    return ret;
}

void* xpci_writeRawDataToFile(unsigned int *par)
{
    int i;
    char fname_0[200];
    FILE *fd_img_0=NULL;    
    int max=0;    
    unsigned imgSize = par[0];
    unsigned burst   = par[1];
    unsigned nbimage_theard = par[2];
    int maxImgBuff = par[3];

    // Variables for Async Reading
    int              fd;
    unsigned int     *imageNumber;

    //**************** Start of async variable set ****************
    //**************** imageNumber ****************                 //Number of the last image acquired
    // Create a new memory object
    fd = shm_open( "/imageNumber", O_RDWR | O_CREAT, 0777 );
    if( fd == -1 ) {
        fprintf( stderr, "Open failed [imageNumber %s()]:%s\n",__func__,
                 strerror( errno ) );
        return -1;
    }

    // Set the memory object's size
    if( ftruncate( fd, sizeof( *imageNumber ) ) == -1 ) {
        fprintf( stderr, "ftruncate [imageNumber %s()]:%s\n",__func__,
                 strerror( errno ) );
        return -1;
    }

    // Map the memory object
    imageNumber = (unsigned int *) mmap( NULL, sizeof( *imageNumber ),
                                         PROT_WRITE,
                                         MAP_SHARED, fd, 0 );
    if( imageNumber == MAP_FAILED ) {
        fprintf( stderr, "imageNumber mmap failed [imageNumber %s()]:%s\n",__func__,
                 strerror( errno ) );
        return -1;
    }

    close(fd);

   //**************** End of async variable set ****************
    imageNumber[0] = 0;

    for (i=0; i<nbimage_theard; i++){
        sprintf(fname_0,"/opt/imXPAD/tmp/burst_%d_img_%d.bin",burst,i);
        fd_img_0 = fopen(fname_0,"wb");
        if(fd_img_0==NULL){
            printf("ERROR => Can not open file < %s >\n",fname_0);
            return -1;
        }
        while(read_pRawBuff_ssd <= write_pRawBuff_ssd && xpci_getResetProcess()==0);
        fwrite(pRawBuff_ssd[i%maxImgBuff],imgSize,1,fd_img_0);
        write_pRawBuff_ssd++;    
        fflush(fd_img_0);    
        fclose(fd_img_0);
      //  printf("write_pRawBuff_ssd = %d\n",write_pRawBuff_ssd);
        imageNumber[0] = i + 1;
        if(xpci_getAbortProcess()){
			 imageNumber[0] = i + 2;
			 break;
		 }
        if(xpci_getResetProcess()){
			 imageNumber[0] = i + 2;
			 break;
		 }
        
        if((read_pRawBuff_ssd - write_pRawBuff_ssd) > max)
			max = read_pRawBuff_ssd - write_pRawBuff_ssd;

    }

    munmap(imageNumber,sizeof( *imageNumber ));
    
    printf("%s() --> Diff ImgRead - ImgWrite max = %d\n",__func__,max);
    
    return NULL;
}




int xpci_loadMemDiag(unsigned modMask,uint16_t type, uint16_t value){
 
     int ret = 0;
    uint16_t *msg;

    if (debugMsg) printf("Doing %s(0x%04x, type =  %d, value = %d)\n",  __func__, modMask,type,value);
    //if (xpci_takeDetector()==-1){return -1;}

    if ( modMask==0)
        return 0;

    // configure subchannel registers
    xpix_imxpadWriteSubchnlReg(modMask, 0, 1);

    msg = malloc(sizeof(MOD_memDiag));
    memcpy(msg,MOD_memDiag,sizeof(MOD_memDiag));
    msg[3] = modMask; // set the mask value
    msg[8] = type; // set the counter value
    msg[9] = value;

    if(xpci_systemType == IMXPAD_S1400){
        ret = xpci_writeCommon_S1400(msg, sizeof(MOD_memDiag),modMask);
    }
    else{
        ret = xpci_writeCommon(msg, sizeof(MOD_memDiag));
    }
    free(msg);
    if (ret){
        printf("ERROR: %s() failed sending the request\n" ,__func__);
        return -1;
    }
    // Now wait to receive ACK message with one message on each channel if necessary
    // We dont check all modules ... only one per channel
    xpci_setHardTimeout(HWTIMEOUT_8SEC); // set a long hardware timeout 8s for this function
    printf("INFO: %s() ... WAIT ...can be long for the data to load ...\n", __func__);
    //sleep(10); // ATTENTION REALLY NECESSARY

    ret = waitCommandReply(modMask, (char*)__func__, 15000);

    xpci_setHardTimeout(HWTIMEOUT_1SEC);//set back default value
    //xpci_freeDetector();
	return ret;
		
}


int xpci_modMemDiag(unsigned modMask,uint16_t type, uint16_t value, uint32_t *data)
{
    if(xpci_loadMemDiag(modMask,type,value)!=0)
        return -1;
    if(xpci_readOneImage(B4, modMask, 7, (void *)data)!=0)
        return -1;    
    return 0;	
	
}


int xpci_modReadADC(unsigned modMask,float *VA,float *VD,float *VT,float *HV)
{
	//ADC Value channel [0] (HV)
	//ADC Value channel [1] (VA)
	//ADC Value channel [2] (VD)
	//ADC Value channel [3] (VT)
	
	const float bin = (3.3/256);
	//const float bin = (2.5/256);
    const float binHV = (bin*(1000000+20000)/20000);
    int	ret, i, j= 0;
    uint16_t *msg;
    int lastMod  = xpci_getLastMod(modMask);
    int nbMod  =  xpci_getModNb(modMask);
    unsigned detData[16*lastMod];
    unsigned mod = 0;

    if (debugMsg) printf("Doing %s(0x%04x)\n",  __func__,modMask);
    if ( modMask==0)
        return 0;
    msg = malloc(sizeof(MOD_readADC));
    memcpy(msg,MOD_readADC,sizeof(MOD_readADC));
    msg[3]  = (uint16_t)modMask; // set the mask value

    // configure subchannel registers
    xpix_imxpadWriteSubchnlReg(modMask, 0, 1);
    //ret = xpci_writeCommon(msg, sizeof(MOD_readConfigG));
    if(xpci_systemType == IMXPAD_S1400){
        ret = xpci_writeCommon_S1400(msg, sizeof(MOD_readADC),modMask);
    }
    else{
        ret = xpci_writeCommon(msg, sizeof(MOD_readADC));
    }
    free(msg);
    if (ret){
        printf("ERROR: %s() failed sending the request\n" ,__func__);
        return -1;

    }
    ret = waitCommandReplyExtended(modMask, (char*)__func__, 10000, detData);
    for(i=0;i<nbMod;i++){
		mod = detData[i*16+1]-1;
		printf("%s()\t mod = %d\n",__func__,mod);
        if((modMask & (1 << mod))==0) continue;
		if(mod < 0 || mod > lastMod) continue;
		if(detData[4+16*mod]==0x8000) HV[mod] = -1; else HV[mod] = (unsigned short)detData[4+16*i]*binHV;
		if(detData[5+16*mod]==0x8000) VA[mod] = -1; else VA[mod] = (unsigned short)detData[5+16*i]*bin;
		if(detData[6+16*mod]==0x8000) VD[mod] = -1; else VD[mod] = (unsigned short)detData[6+16*i]*bin;
		if(detData[7+16*mod]==0x8000) VT[mod] = -1; else VT[mod] = (unsigned short)detData[7+16*i]*bin;
	}    
    return ret;	
}


int xpci_modSetHV(unsigned modMask,uint8_t value)
{
    int	ret, i, j= 0;
    uint16_t *msg;
    int lastMod  = xpci_getLastMod(modMask);
    int nbMod  =  xpci_getModNb(modMask);
    unsigned detData[16*lastMod];
    unsigned mod = 0;

    if (debugMsg) printf("Doing %s(0x%04x)\n",  __func__,modMask);
    if ( modMask==0)
        return 0;
    msg = malloc(sizeof(MOD_setHV));
    memcpy(msg,MOD_setHV,sizeof(MOD_setHV));
    msg[3]  = (uint16_t)modMask; // set the mask value
    msg[8] = value;

    // configure subchannel registers
    xpix_imxpadWriteSubchnlReg(modMask, 0, 1);
    //ret = xpci_writeCommon(msg, sizeof(MOD_readConfigG));
    if(xpci_systemType == IMXPAD_S1400){
        ret = xpci_writeCommon_S1400(msg, sizeof(MOD_setHV),modMask);
    }
    else{
        ret = xpci_writeCommon(msg, sizeof(MOD_setHV));
    }
    free(msg);
    if (ret){
        printf("ERROR: %s() failed sending the request\n" ,__func__);
        return -1;

    }
    ret = waitCommandReplyExtended(modMask, (char*)__func__, 10000, detData);
    return ret;	
}


