#ifndef IMXPAD_CALIB
#define IMXPAD_CALIB

#if defined(__cplusplus)
extern "C" 
{

#endif

  int imxpad_saveDaclMatrix(unsigned modMask, char *path, unsigned *daclMatrix);
  int imxpad_readDataMatrix(FILE *fdacl, unsigned modMask, unsigned *daclmatrix);
  int imxpad_detSaveDaclMatrix_S540(unsigned modMask, unsigned *daclmatrix);
  int imxpad_detSaveDaclMatrix_S420(unsigned modMask, unsigned *daclmatrix);
  int imxpad_detSaveDaclMatrix_S140(unsigned modMask, unsigned *daclmatrix);
  int imxpad_detSaveDaclMatrix_S70(unsigned modMask, unsigned *daclmatrix);
  int imxpad_detSaveDaclMatrix_S1400(unsigned modMask, unsigned *daclmatrix);
  int imxpad_uploadDaclMatrix(unsigned modMask, unsigned *daclmatrix);
  int imxpad_fileUploadDaclMatrix(char *fpath, unsigned modMask);
  int imxpad_fileCreateConfigG(char *fpath, unsigned modMask);
  int imxpad_fileUploadConfigG(char *fpath);
  int imxpad_scanDACL(unsigned modMask, unsigned Texp, char *path);
  int imxpad_processDaclScanData(int calibType, unsigned modMask, char *dirpath, unsigned *daclMatrix,unsigned int maxSCurve);
  int imxpad_processDaclScanDataBEAM(int calibType, unsigned modMask, char *dirpath, unsigned *daclMatrix);
  unsigned imxpad_processDaclProfileOTN(unsigned *daclProfile);
  unsigned imxpad_processDaclProfileBEAM(unsigned *daclProfile,unsigned int maxSCurve);
  unsigned imxpad_processOTNiteration(unsigned modMask, unsigned *daclMatrix, uint16_t *image);
  int imxpad_scanITHL(unsigned modMask, unsigned Texp, unsigned ithl_min, unsigned ithl_max, char *path);
  int imxpad_processIthlScanData(unsigned modMask, char *dirpath, unsigned ithl_min, unsigned ithl_max, unsigned *ithlval);
  int imxpad_processIthlScanDataOTN(unsigned modMask, char *dirpath, unsigned ithl_min, unsigned ithl_max, unsigned *ithlval);
  int imxpad_processIthlScanDataBEAM(unsigned modMask, char *dirpath, unsigned ithl_min, unsigned ithl_max, unsigned *ithlval);
  int imxpad_searchIthlValues_noise(unsigned modMask, unsigned *ithl_noisematrix, unsigned *ithlimg);
  int imxpad_calibration_OTN_slow(unsigned modMask, char *path, unsigned iterations);
  int imxpad_calibration_OTN_medium(unsigned modMask, char *path, unsigned iterations);
  int imxpad_calibration_OTN_fast(unsigned modMask, char *path, unsigned iterations);
  int imxpad_calibration_OTN(unsigned modMask, char *path, unsigned iterations, unsigned itune, unsigned imfp);
  int imxpad_calibration_BEAM(unsigned modMask, char *path, unsigned Texp, unsigned ithl_max, unsigned itune, unsigned imfp,unsigned int maxSCurve);
  int imxpad_uploadCalibration(unsigned modMask, char *path);

  unsigned imxpad_desableNoisyPixels(unsigned modMask, unsigned *daclMatrix, uint16_t *image); //fred

int imxpad_scanDACL_pulse(unsigned modMask, unsigned nbPulse, char *path);
int imxpad_calibration_OTN_pulse(unsigned modMask, char *path, unsigned iterations, unsigned itune, unsigned imfp);
#ifdef __cplusplus
}
#endif
#endif
