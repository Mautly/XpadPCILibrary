#ifndef XPCI_IMXPAD
#define XPCI_IMXPAD

#if defined(__cplusplus)
extern "C" 
{

#endif

int imxpad_raw2data(int modMask, void *pOldData, void *pNewData);
int imxpad_raw2data_16bits(int modMask, uint16_t *pOldData, uint16_t *pNewData);
int imxpad_raw2data_32bits(int modMask, uint16_t *pOldData, uint32_t *pNewData);
int imxpad_extract2BImgData(int modMask, uint16_t *oldImg, uint16_t *newImg);
int imxpad_extract4BImgData(int modMask, uint16_t *oldImg, uint32_t *newImg);
int imxpad_extract2BImgData_S140(int modMask, uint16_t *oldImg, uint16_t *newImg);
int imxpad_extract4BImgData_S140(int modMask, uint16_t *oldImg, uint32_t *newImg);
int imxpad_extract2BImgData_S70(int modMask, uint16_t *oldImg, uint16_t *newImg);
int imxpad_extract4BImgData_S70(int modMask, uint16_t *oldImg, uint32_t *newImg);

// added by fred for the FLI detector
int imxpad_extract2BImgData_S420(int modMask, uint16_t *oldImg, uint16_t *newImg);
int imxpad_extract4BImgData_S420(int modMask, uint16_t *oldImg, uint32_t *newImg);


int imxpad_incrITHL(unsigned modMask);
int imxpad_decrITHL(unsigned modMask);

int imxpad_uploadExpWaitTimes(unsigned modMask, unsigned *pWaitTime, unsigned size);

int imxpad_raw2data_16bits_v2(int modMask, uint16_t* oldImg, uint16_t** newImg,int imgNumber);
int imxpad_extract2BImgData_S1400_v2(int modMask, uint16_t *oldImg, uint16_t **newImg);
int imxpad_extract2BImgData_S1400(int modMask, uint16_t *oldImg, uint16_t *newImg);

int imxpad_writeFile2B_ALBA_S1400(unsigned modMask,char *fileName , uint16_t *newImg);
int imxpad_writeFile4B_ALBA_S1400(unsigned modMask,char *fileName , uint32_t *newImg);

int imxpad_raw_file_to_images(enum IMG_TYPE type, unsigned modMask, char *fpathOut, int startImg, int stopImg, int burstNumber);
int imxpad_raw_file_to_buffer(enum IMG_TYPE type, unsigned modMask, void *pRawBuffOut , int numImageToAcquire, int burstNumber);

#ifdef __cplusplus
}
#endif
#endif
