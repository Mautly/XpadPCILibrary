// ******************************************************
//                   imxpad_calibration.c
// 
// Functions to upload and create calibration files.
//
// Author: Arkadiusz Dawiec
// Author: Frédéric BOMPARD
// Created: 27/03/2012
// *****************************************************
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

#include "xpci_interface.h"
#include "xpci_interface_expert.h"
#include "xpci_calib_imxpad.h"

// system type
extern int xpci_systemType;

#define CALIB_OTN  0
#define CALIB_BEAM 1

// ---------------------------------------------------------------------
// function to save dacl matrix in a file
int imxpad_saveDaclMatrix(unsigned modMask, char *path, unsigned *daclMatrix){
    int pos = strlen(path);
    char fname[pos+15];
    FILE *wfile;
    int i, j = 0;
    int row=0;

    int modNb = xpci_getModNb(modMask);
    int lastMod    = xpci_getLastMod(modMask);
    int FirstMod   = xpci_getFirstMod(modMask);


    sprintf(fname, "%s/DACL_matrix.dat", path);

    // open file for writing
    wfile = fopen(fname, "w");
    if (wfile==NULL){
        printf("%s() ERROR: faield to open file %s\n", __func__, fname);
        return -1;
    }

    for(i=FirstMod;i<lastMod;i++){
        if((modMask & (1 << i)))
        {
            for(row=0;row<120;row++)
            {
                for(j=0;j<560;j++){
                    if(j<559)
                        fprintf(wfile,"%u ", *(daclMatrix+i*560*120+row*560+j));
                    else
                        fprintf(wfile,"%u\n", *(daclMatrix+i*560*120+row*560+j));
                }
            }
        }
        else
        {
            for(row=0;row<120;row++)
            {
                for(j=0;j<560;j++){
                    if(j<559)
                        fprintf(wfile,"%u ", 0);
                    else
                        fprintf(wfile,"%u\n", 0);
                }
            }
        }
    }
    // close file
    fclose(wfile);
    return 0;
}

// ---------------------------------------------------------------------
// function to read dacl matrix from file
int imxpad_readDataMatrix(FILE *fdacl, unsigned modMask, unsigned *daclmatrix){

    char fline[7000]; // max length of a line is 560*10 (max 10 digits numbers) + 560 (space delimiters)
    char *pLine, *pSpace;
    int i, j = 0;

    int modules_nb = xpci_getModNb(modMask);

    unsigned detHeight=120*modules_nb;
    
    

    // go back to the begining of the file
    rewind(fdacl);

    for(i=0;i<detHeight; i++){
        if(fgets(fline,sizeof(fline), fdacl) != NULL ){
            // set pointer at the beginning of the line
            pLine=&fline[0];
            for(j=0; j<560; j++){
                //sscanf(pLine,"%"SCNu16" %*s", &daclmatrix[i*560+j]);
                sscanf(pLine,"%u%*s", &daclmatrix[i*560+j]);
                // find next space character
                pSpace=strchr(pLine,' ');
                // set pointer after the space
                pSpace++;
                pLine=pSpace;
            }
        }
        else{
            printf("%s() ERROR: enexpected end of file (line %d)\n", __func__, i);
            return  -1;
        }
    }
    return 0;
}

// ---------------------------------------------------------------------
// function to upload a dacl matrix to S540 detector
// in the S540 type each pair of modules connected to the same
// FPGA chip is swapped (i.e. 2 & 1, 4 & 3, 6 & 5, 8 & 7) 
// in order to properly upload date the rowOffset parameter is
// implemented (offset of +/- 120 rows)
int imxpad_detSaveDaclMatrix_S540(unsigned modMask, unsigned *daclmatrix){
    int module = 0;
    int topRow = 0;
    int row = 0;
    int chip = 0;
    int rowOffset = 0;
    unsigned localMask = 0;
    int ret = 0;

    for(module=0; module<8; module++){
        rowOffset = (module%2) ? (-120) : 120;
        if((modMask>>module)&1){
            topRow = (module+1)*120-1+rowOffset;
            localMask=0x01<<module;
            for(row=0; row<120; row++){
                for(chip=0; chip<7; chip++){
				    if(xpci_getAbortProcess())	return 1;
                    printf("\r\t  loading line %d of 840", row*7+chip+1);
                    fflush(stdout);
                    if(xpci_getAbortProcess()) return 1;
                    // values are uploaded from row 120 to row 0
                    if (xpci_modSaveConfigL(localMask, 0, chip, 119-row, &daclmatrix[(topRow-row)*560+chip*80])!=0){
                        printf("\n%s() ERROR: failed to save data in detetor's onboard memory: module 0x%02x chip %d line %d\n", __func__, modMask, chip, row);
                        ret = -1;
                    }
                }
            }
            printf("\n");
        }
    }
    return ret;
}




// ---------------------------------------------------------------------
// function to upload a dacl matrix to S540 detector
// in the S540 type each pair of modules connected to the same
// FPGA chip is swapped (i.e. 2 & 1, 4 & 3, 6 & 5, 8 & 7) 
// in order to properly upload date the rowOffset parameter is
// implemented (offset of +/- 120 rows)
int imxpad_detSaveDaclMatrix_S1400(unsigned modMask, unsigned *daclmatrix){
    int module = 0;
    int topRow = 0;
    int row = 0;
    int chip = 0;
    int rowOffset = 0;
    unsigned localMask = 0;
    int ret = 0;

    int modules_nb = xpci_getModNb(modMask);
    int lastMod    = xpci_getLastMod(modMask);
    int FirstMod   = xpci_getFirstMod(modMask);

    for(module=FirstMod; module<lastMod; module++){
        if((modMask>>module)&1){
            topRow = (module+1)*120-1;
            localMask=0x01<<module;
            for(row=0; row<120; row++){
                for(chip=0; chip<7; chip++){
                    printf("\r\t  loading line %d of 840", row*7+chip+1);
                    fflush(stdout);
                    if(xpci_getAbortProcess()) return 1;
                    // values are uploaded from row 120 to row 0
                    if (xpci_modSaveConfigL(localMask, 0, chip, 119-row, &daclmatrix[(topRow-row)*560+chip*80])!=0){
                        printf("\n%s() ERROR: failed to save data in detetor's onboard memory: module 0x%02x chip %d line %d\n", __func__, localMask, chip, row);
                        ret = -1;
                    }
                }
            }
        }
        printf("\n");
    }
    return ret;
}


int imxpad_detSaveDaclMatrix_S420(unsigned modMask, unsigned *daclmatrix){
    int module = 0;
    int topRow = 0;
    int row = 0;
    int chip = 0;
    int rowOffset = 0;
    unsigned localMask = 0;
    int ret = 0;
    int firstMod = xpci_getFirstMod(modMask);

    for(module=firstMod; module<6+firstMod; module++){
        rowOffset = (module%2) ? (-120) : 120;
        if((modMask>>(module))&1){
            topRow = (module+1-firstMod)*120-1+rowOffset;
            localMask=0x01<<(module);
            for(row=0; row<120; row++){
                for(chip=0; chip<7; chip++){
					if(xpci_getAbortProcess())	return 1;
                    printf("\r\t  loading line %d of 840", row*7+chip+1);
                    fflush(stdout);
                    if(xpci_getAbortProcess()) return 1;
                    // values are uploaded from row 120 to row 0
                    if (xpci_modSaveConfigL(localMask, 0, chip, 119-row, &daclmatrix[(topRow-row)*560+chip*80])!=0){
                        printf("\n%s() ERROR: failed to save data in detetor's onboard memory: module 0x%02x chip %d line %d\n", __func__, modMask, chip, row);
                        ret = -1;
                    }
                }
            }
            printf("\n");
        }
    }
    return ret;
}


// ---------------------------------------------------------------------
// function to upload a dacl matrix to S140 detector
// in the S540 detector the second module is horozontaly inverted (row 1 <-> 120)
// and also each chip is vertically inverted (col 1 <-> 80)
int imxpad_detSaveDaclMatrix_S140(unsigned modMask, unsigned *daclmatrix){
    int module = 0;
    int row = 0;
    int chip = 0;
    int col = 0;
    int rowOffset = 0;
    unsigned localMask = 0;
    unsigned localdacl[80];
    int ret = 0;

    for(module=0; module<2; module++){
        if((modMask>>module)&1){
            localMask=0x01<<module;
            printf("\nmodule %d\n",module);

            for(row=0; row<120; row++){
                for(chip=0; chip<7; chip++){
                    printf("\r\t  loading line %d of 840", row*7+chip+1);
                    fflush(stdout);
                    if(xpci_getAbortProcess()) return 1;
                    // module 1 can be send directly (now data rearragement is required)
                    if (module==0) {
                        if (xpci_modSaveConfigL(localMask, 0, chip, 119-row, &daclmatrix[(119-row)*560+chip*80])!=0){
                            printf("\n%s() ERROR: failed to save data in detetor's onboard memory: module 0x%02x chip %d line %d\n", __func__, localMask, chip, row);
                            ret = -1;
                        }
                    }
                    else {
                        // module one has vertical swap across the module and vertical swaps in the chips
                        for(col=0; col<80; col++)
                            localdacl[79-col]=daclmatrix[(120+row)*560+chip*80+col];

                        if (xpci_modSaveConfigL(localMask, 0, chip, 119-row, &localdacl[0])!=0){
                            printf("\n%s() ERROR: failed to save data in detetor's onboard memory: module 0x%02x chip %d line %d\n", __func__, localMask, chip, row);
                            ret = -1;
                        }
                    }
                }
            }
            printf("\n");
        }
    }
    return ret;
}

// ---------------------------------------------------------------------
// function to upload a dacl matrix to S70 detector
int imxpad_detSaveDaclMatrix_S70(unsigned modMask, unsigned *daclmatrix){
    int row = 0;
    int chip = 0;
    int ret = 0;

    if(modMask&1){
        for(row=0; row<120; row++){
            for(chip=0; chip<7; chip++){
			    if(xpci_getAbortProcess())
					return 1;
                printf("\r\t  loading line %d of 840", row*7+chip+1);
                fflush(stdout);
                if(xpci_getAbortProcess()) return 1;
                // values are uploaded from row 120 to row 0
                if (xpci_modSaveConfigL(modMask, 0, chip, 119-row, &daclmatrix[(119-row)*560+chip*80])!=0){
                    printf("\n%s() ERROR: failed to save data in detetor's onboard memory: module 0x%02x chip %d line %d\n", __func__, modMask, chip, row);
                    ret = -1;
                }
            }
        }
        printf("\n");
    }
    return ret;
}

// ---------------------------------------------------------------------
// function to upload a dacl matrix to the detector
int imxpad_uploadDaclMatrix(unsigned modMask, unsigned *daclmatrix){

   // xpci_clearAbortProcess();
   usleep(100000);
    xpci_modGlobalAskReady(modMask);
    // save daclmatrix in the detector accordingly to the system type
    switch(xpci_systemType){
    case IMXPAD_S70:
        imxpad_detSaveDaclMatrix_S70(modMask, daclmatrix);
        break;
    case IMXPAD_S140:
        imxpad_detSaveDaclMatrix_S140(modMask, daclmatrix);
        break;
    case IMXPAD_S420:
        imxpad_detSaveDaclMatrix_S420(modMask, daclmatrix);
        break;
    
     //   imxpad_detSaveDaclMatrix_S540(modMask, daclmatrix);
    //    break;
    case IMXPAD_S540:
    case IMXPAD_S700:
    case IMXPAD_S1400:
        imxpad_detSaveDaclMatrix_S1400(modMask, daclmatrix);
        break;
    default:
        printf("%s() ERROR: unknown system type\n", __func__);
        break;
    }
	if(xpci_getAbortProcess())	return 1;
    // upload previously saved data to the XPAD chips
    if(xpci_modDetLoadConfig(modMask, 0)!=0){
        printf("%s() ERROR: failed to load calibration data from the onboard memory to the XPAD chips\n",__func__);
        return -1;
    }
    if(xpci_getAbortProcess())	return 1;
   // xpci_clearAbortProcess();
    return 0;
}

// ---------------------------------------------------------------------
// upload dacl matrix from the file
int imxpad_fileUploadDaclMatrix(char *fpath, unsigned modMask){
    FILE *fdacl;
    int modNb = xpci_getModNb(modMask);

    unsigned *pDACL = malloc(modNb*120*560*sizeof(unsigned));

    // read file
    printf("reading dacl matrix file\n");
    if ((fdacl=fopen(fpath, "r"))==NULL){
        printf("%s() ERROR: failed to open file %s\n", __func__, fdacl);
        return -1;
    }
    else{
        // read DACL matrix file to a local buffer
        if(imxpad_readDataMatrix(fdacl, modMask, pDACL)!=0){
            fclose(fdacl);
            return -1;
        }
        fclose(fdacl);
        // upload matrix to the detector
        imxpad_uploadDaclMatrix(modMask, pDACL);
    }
	if(xpci_getAbortProcess())	return 1;
    printf("\n\n");
    return 0;
}

// ---------------------------------------------------------------------
// function to create configg file
// all config registers are read and saved in a file
int imxpad_fileCreateConfigG(char *fpath, unsigned modMask){

    int modNb = xpci_getModNb(modMask);
    int firstMod = xpci_getFirstMod(modMask);
    int lastMod    = xpci_getLastMod(modMask);



    unsigned *pRegCmosDsbl;
    unsigned *pRegAmpTp;
    unsigned *pRegVadj;
    unsigned *pVref;
    unsigned *pRegImfp;
    unsigned *pRegIota;
    unsigned *pRegIpre;
    unsigned *pRegIthl;
    unsigned *pRegItune;
    unsigned *pRegIbuffer;
    FILE *wfile;
    int i,j = 0;

    // allocate buffers for global registers
    pRegCmosDsbl = malloc(7*lastMod*sizeof(unsigned));
    pRegAmpTp = malloc(7*lastMod*sizeof(unsigned));
    pRegVadj = malloc(7*lastMod*sizeof(unsigned));
    pVref = malloc(7*lastMod*sizeof(unsigned));
    pRegImfp = malloc(7*lastMod*sizeof(unsigned));
    pRegIota = malloc(7*lastMod*sizeof(unsigned));
    pRegIpre = malloc(7*lastMod*sizeof(unsigned));
    pRegIthl = malloc(7*lastMod*sizeof(unsigned));
    pRegItune = malloc(7*lastMod*sizeof(unsigned));
    pRegIbuffer = malloc(7*lastMod*sizeof(unsigned));

     usleep(5000);
    // read configg registers from the detector
    // CMOS DSBL
    if(xpci_modReadConfigG(modMask, 0x7f, CMOS_DSBL, pRegCmosDsbl) != 0){
        printf("%s() ERROR: failed to read global register (CMOS_DSBL)\n", __func__);
        return -1;
    }
    // AMPTP
    if(xpci_modReadConfigG(modMask, 0x7f, AMP_TP, pRegAmpTp) != 0){
        printf("%s() ERROR: failed to read global register (AMP_TP)\n", __func__);
        return -1;
    }
    // VADJ
    if(xpci_modReadConfigG(modMask, 0x7f, VADJ, pRegVadj) != 0){
        printf("%s() ERROR: failed to read global register (VADJ)\n", __func__);
        return -1;
    }
    // VREF
    if(xpci_modReadConfigG(modMask, 0x7f, VREF, pVref) != 0){
        printf("%s() ERROR: failed to read global register (VREF)\n", __func__);
        return -1;
    }
    // IMFP
    if(xpci_modReadConfigG(modMask, 0x7f, IMFP, pRegImfp) != 0){
        printf("%s() ERROR: failed to read global register (IMFP)\n", __func__);
        return -1;
    }
    // IOTA
    if(xpci_modReadConfigG(modMask, 0x7f, IOTA, pRegIota) != 0){
        printf("%s() ERROR: failed to read global register (IOTA)\n", __func__);
        return -1;
    }
    // IPRE
    if(xpci_modReadConfigG(modMask, 0x7f, IPRE, pRegIpre) != 0){
        printf("%s() ERROR: failed to read global register (IPRE)\n", __func__);
        return -1;
    }
    // ITHL
    if(xpci_modReadConfigG(modMask, 0x7f, ITHL, pRegIthl) != 0){
        printf("%s() ERROR: failed to read global register (ITHL)\n", __func__);
        return -1;
    }
    // ITUNE
    if(xpci_modReadConfigG(modMask, 0x7f, ITUNE, pRegItune) != 0){
        printf("%s() ERROR: failed to read global register (ITUNE)\n", __func__);
        return -1;
    }
    // IBUFFER
    if(xpci_modReadConfigG(modMask, 0x7f, IBUFFER, pRegIbuffer) != 0){
        printf("%s() ERROR: failed to read global register (IBUFFER)\n", __func__);
        return -1;
    }

    printf("Read all global configs\n"); fflush(stdout);

    // open file for writing
    wfile = fopen(fpath, "w");
    if (wfile==NULL){
        printf("%s ERROR: while opening file %s\n", __func__, fpath);
        return -1;
    }
    // write data to .dat file
    //CMOS DSBL
    for (i=firstMod; i<lastMod;i++){
        if((modMask & (1<<i))==0) continue;
        //fprintf(wfile,"%"SCNu16" %"SCNu16, 0x01<<i,CMOS_DSBL);
        fprintf(wfile,"%u %u", 0x01<<i,CMOS_DSBL);
        for(j=0; j<7;j++)
            fprintf(wfile," %"SCNu16, *(pRegCmosDsbl+i*7+j));
        fprintf(wfile,"\n");
    }
    // AMP_TP
    for (i=firstMod; i<lastMod;i++){
        if((modMask & (1<<i))==0) continue;
        fprintf(wfile,"%u %u", 0x01<<i,AMP_TP);
        for(j=0; j<7;j++)
            fprintf(wfile," %"SCNu16, *(pRegAmpTp+i*7+j));
        fprintf(wfile,"\n");
    }
    // VADJ
    for (i=firstMod; i<lastMod;i++){
        if((modMask & (1<<i))==0) continue;
        fprintf(wfile,"%u %u", 0x01<<i,VADJ);
        for(j=0; j<7;j++)
            fprintf(wfile," %"SCNu16, *(pRegVadj+i*7+j));
        fprintf(wfile,"\n");
    }
    // VREF
    for (i=firstMod; i<lastMod;i++){
        if((modMask & (1<<i))==0) continue;
        fprintf(wfile,"%u %u", 0x01<<i,VREF);
        for(j=0; j<7;j++)
            fprintf(wfile," %"SCNu16, *(pVref+i*7+j));
        fprintf(wfile,"\n");
    }
    // IMFP
    for (i=firstMod; i<lastMod;i++){
        if((modMask & (1<<i))==0) continue;
        fprintf(wfile,"%u %u", 0x01<<i,IMFP);
        for(j=0; j<7;j++)
            fprintf(wfile," %"SCNu16, *(pRegImfp+i*7+j));
        fprintf(wfile,"\n");
    }
    // IOTA
    for (i=firstMod; i<lastMod;i++){
        if((modMask & (1<<i))==0) continue;
        fprintf(wfile,"%u %u", 0x01<<i,IOTA);
        for(j=0; j<7;j++)
            fprintf(wfile," %"SCNu16, *(pRegIota+i*7+j));
        fprintf(wfile,"\n");
    }
    // IPRE
    for (i=firstMod; i<lastMod;i++){
        if((modMask & (1<<i))==0) continue;
        fprintf(wfile,"%u %u", 0x01<<i,IPRE);
        for(j=0; j<7;j++)
            fprintf(wfile," %"SCNu16, *(pRegIpre+i*7+j));
        fprintf(wfile,"\n");
    }
    // ITHL
    for (i=firstMod; i<lastMod;i++){
        if((modMask & (1<<i))==0) continue;
        fprintf(wfile,"%u %u", 0x01<<i,ITHL);
        for(j=0; j<7;j++)
            fprintf(wfile," %"SCNu16, *(pRegIthl+i*7+j));
        fprintf(wfile,"\n");
    }
    // ITUNE
    for (i=firstMod; i<lastMod;i++){
        if((modMask & (1<<i))==0) continue;
        fprintf(wfile,"%u %u", 0x01<<i,ITUNE);
        for(j=0; j<7;j++)
            fprintf(wfile," %"SCNu16, *(pRegItune+i*7+j));
        fprintf(wfile,"\n");
    }
    // IBUFFER
    for (i=firstMod; i<lastMod;i++){
        if((modMask & (1<<i))==0) continue;
        fprintf(wfile,"%u %u", 0x01<<i,IBUFFER);
        for(j=0; j<7;j++)
            fprintf(wfile," %"SCNu16, *(pRegIbuffer+i*7+j));
        fprintf(wfile,"\n");
    }
    // close file
    fclose(wfile);

    // free all buffers
    free(pRegCmosDsbl);
    free(pRegAmpTp);
    free(pRegVadj);
    free(pVref);
    free(pRegImfp);
    free(pRegIota);
    free(pRegIpre);
    free(pRegIthl);
    free(pRegItune);
    free(pRegIbuffer);

    return 0;
}


// ---------------------------------------------------------------------
// function to read a configg file and upload values to the detector
int imxpad_fileUploadConfigG(char *fpath){
    FILE *rdfile;
    char fline[100];
    unsigned modMask = 0;
    unsigned chipMask = 0;
    unsigned chip = 0;
    unsigned reg = 0;
    unsigned m_reg=0;
    unsigned regVal[7];
    int i;
    int flag_config;
    int ret = 0;    
    xpci_clearAbortProcess();    
    // open file for reading
    if ((rdfile=fopen(fpath, "r"))==NULL){
        printf("%s() ERROR: failed to open file %s\n", __func__, fpath);
        return -1;
    }

    // read lines, one by line
    while (fgets(fline,sizeof(fline), rdfile) != NULL ){
        sscanf(fline,"%u %u %u %u %u %u %u %u %u", &modMask, &reg, &regVal[0], &regVal[1], &regVal[2], &regVal[3], &regVal[4], &regVal[5], &regVal[6]);

        if(m_reg != reg)
        {
            m_reg=reg;
            printf("\nLoad global register 0x%x\n",reg);
            //printf("\nLoad global register %u\n",reg);
        }
        flag_config = 1;
        for(i=1;i<7;i++)
        {
				if(regVal[i] != regVal[i-1]){
					flag_config = 0;	
					break;
				}		
		}
        
        if(flag_config == 1){ // if all config G are egal for the module
			printf(".......");
			if(xpci_modLoadConfigG(modMask, 0x7F, reg, regVal[0]) != 0){
					printf("%s() ERROR: failed to write global register\n", __func__);
					ret = -1;
					break;
				}
				if(xpci_getAbortProcess()) return 1;        
	    }
	    else{
			for(chip=0;chip<7;chip++){
				printf(".");
				//printf("%u ", regVal[chip]);
				fflush(stdout);
				chipMask = 0x01<<chip;
				if(xpci_modLoadConfigG(modMask, chipMask, reg, regVal[chip]) != 0){
					printf("%s() ERROR: failed to write global register\n", __func__);
					ret = -1;
					break;
				}
				if(xpci_getAbortProcess()) return 1;
			}
		}
        printf("\n");
    }

    fclose(rdfile);
    return ret;
}

// ---------------------------------------------------------------------
// function to make a dacl scan
// all images are read in 16 bits format
int imxpad_scanDACL(unsigned modMask, unsigned Texp, char *path){
    int pos = strlen(path);
    struct stat status;
    unsigned daclVal = 0;
    char fname[pos+15];
    FILE *wfile;
    uint16_t **daclImg;
    uint16_t *img;
    int modNb = xpci_getModNb(modMask);
    int firstMod = xpci_getFirstMod(modMask);
    int lastMod    = xpci_getLastMod(modMask);
    int row, col = 0;
	xpci_modGlobalAskReady(modMask);
    // allocate buffer for dacl image
    daclImg = malloc(sizeof(uint16_t *));
    daclImg[0]=malloc(120*560*lastMod*sizeof(uint16_t));
     unsigned ovf_time = 4000;

   // if(xpci_systemType == IMXPAD_S1400)
	//	ovf_time = 0xFFFFFFFF;

    // check if path is not an empty string
    if(pos == 0){
        printf("%s() ERROR: DACL scan directory path cannot be an empty string.\n", __func__);
        free(daclImg[0]);
        return -1;
    }

    // remove '/' char from the end of the string if exist
    while(path[pos-1]=='/'){
        path[pos-1]=NULL;
        pos = strlen(path);
    }

    // check the path and create a directory
    if(stat(path, &status)==0){
        if(S_ISREG(status.st_mode)){
            printf("%s() ERROR: %s is an existing file.\n", __func__, path);
            free(daclImg[0]);
            return -1;
        }
        if(S_ISDIR(status.st_mode))
            printf("%s() ERROR: %s is an existing folder. Possible file overwriting.\n", __func__, path);
    }
    else{
        if(mkdir(path, S_IRWXU |  S_IRWXG |  S_IRWXO)!=0){
            printf("%s.\n", strerror(errno));
            free(daclImg[0]);
            return -1;
        }
    }

    // configure exposure parameters (images erad in 16 bits format)
    if (xpci_modExposureParam_internal(modMask, Texp, 5000, 0, 0, ovf_time, 0, 0, 0, 1, 3, 0, 0, 0, 0, 0, 0)!= 0)
        printf("%s() ERROR: failed to send exposure parameters\n", __func__);

    // start scan
    for(daclVal=0; daclVal<64; daclVal++){   
		int ret;
        if(xpci_getAbortProcess())
            return 1;
		xpci_modGlobalAskReady(modMask);
        printf("\nScan DACL %d/63 steps\n\n",daclVal);
        fflush(stdout);
        // send flat dacl
        if(xpci_modLoadFlatConfig(modMask, 0x7f, daclVal*8+1)!=0){
            printf("%s() ERROR: failed to send flat config %d (DACL=%d). DACL scan aborted.\n", __func__, daclVal*8+1, daclVal);
            free(daclImg[0]);
            return -1;
        }
            // configure exposure parameters (images erad in 16 bits format)
        xpci_modGlobalAskReady(modMask);
		if (xpci_modExposureParam_internal(modMask, Texp, 5000, 0, 0, ovf_time, 0, 0, 0, 1, 3, 0, 0, 0, 0, 0, 0)!= 0){
				printf("%s() ERROR: failed to send exposure parameters\n", __func__);
				xpci_modGlobalAskReady(modMask);
		}

		ret = xpci_getImgSeq(B2, modMask, 7, 1, (void *)daclImg, 0, 0, 0, 0);
        // expose and read
        if(ret!=0){
            printf("%s() ERROR: failed to acquire an image. DACL scan aborted.\n", __func__);
            free(daclImg[0]);
            if(ret == 1)
				xpci_setAbortProcess();
            return ret;
        }

        //assign pointer
        sprintf(fname, "%s/DACL_%d.dat", path,daclVal);
        wfile = fopen(fname, "w");
        if (wfile==NULL){
            printf("%s() ERROR: failed to open file %s\n", __func__, fname);
            free(daclImg[0]);
            return -1;
        }

        img = *daclImg;
        // write data to .dat file
        for (row=0; row<(lastMod*120); row++){ // lines
            for (col=0; col<560; col++) // words
                if(col<559)
                    fprintf(wfile, "%"SCNu16" ", *(img+row*560+col));
                else
                    fprintf(wfile, "%"SCNu16, *(img+row*560+col));
            if(row<(lastMod*120-1))
                fprintf(wfile, "\n");
        }//
        fclose(wfile);
    }
    printf("\n");
    fflush(stdout);

    free(daclImg);
    return 0;
}


// ---------------------------------------------------------------------
// function to process dacl scan data
// for every pixel find a dacl value fro which pixel starts to count
int imxpad_processDaclScanData(int calibType, unsigned modMask, char *dirpath, unsigned *daclMatrix,unsigned int maxSCurve){
    FILE *rdfile;
    int pos = strlen(dirpath);
    char fname[pos+15];
    struct stat path_status;
    int i,j =0;
    int modNb = xpci_getModNb(modMask);
    unsigned **daclscan_img;
    unsigned *daclProfile=malloc(64*sizeof(unsigned));
    unsigned daclValue;
    int      firstMod =xpci_getFirstMod(modMask);
    int      lastMod    = xpci_getLastMod(modMask);
    int noDetect=0; 
    int maxSCurve_tmp=maxSCurve;
    int x,y;

    daclscan_img = malloc(64*sizeof(unsigned *));
    for(i=0; i<64; i++)
        daclscan_img[i] = malloc(120*560*lastMod*sizeof(unsigned));

    // check that directory exist
    if(stat(dirpath, &path_status)==0){
        if(!S_ISDIR(path_status.st_mode)){
            printf("%s() ERROR: %s is not a directory.\n", __func__, dirpath);
            return -1;
        }
    }
    else{
        printf("%s() ERROR: %s directory does not exist.\n", __func__, dirpath);
        return -1;
    }

    // open 64 scan files
    for(i=0; i<64; i++){

        if(xpci_getAbortProcess())
            return 1;

        // build name string
        sprintf(fname, "%s/DACL_%d.dat", dirpath, i);
        // open file for reading
        if ((rdfile=fopen(fname, "r"))==NULL){
            printf("%s() ERROR: failed to open file %s.\n", __func__, fname);
            return -1;
        }
        //read file to the array
        if(imxpad_readDataMatrix(rdfile, modMask, daclscan_img[i])!=0){
            printf("%s ERROR: failed to read file %s.\n", __func__, fname);
            return -1;
        }
        fclose(rdfile);
    }

    for(j=(120*560*firstMod);j<(120*560*lastMod);j++){

        if(xpci_getAbortProcess())
            return 1;
            
        x = j/560;
        y = j-(x*560);

        // create dacl profile
        for(i=0;i<64;i++)
            *(daclProfile+i)=daclscan_img[i][j];
        // analyze dacl profile
        if(calibType==CALIB_OTN){
            *(daclMatrix+j)=imxpad_processDaclProfileOTN(daclProfile);
        }
        else if(calibType==CALIB_BEAM){
			    if(((y%80 == 0) || (y%80 == 79)) && (y > 1) )
					maxSCurve_tmp = maxSCurve*3;
				else
					maxSCurve_tmp = maxSCurve;
					
            *(daclMatrix+j)=imxpad_processDaclProfileBEAM(daclProfile,maxSCurve_tmp);
		}

    }
    printf("\n");

    //release memory
    free(daclProfile);
    //free(daclMatrix);
    for(i=0;i<64;i++)
        free(daclscan_img[i]);

    return 0;
}

// ---------------------------------------------------------------------
// function to analyze dacl profilefor Over-The-Noise calibration
unsigned imxpad_processDaclProfileOTN(unsigned *daclProfile){
    int i = 0;
    unsigned daclValue=0;
    unsigned ret = 0;
    unsigned int max = 0;
    int max_detect=0;


    // detect the noise peak (only count above 5 in order not be affected by cosmics)
    max = 0 ;
    for (i=0; i<64; i++){

        if(*(daclProfile+i)>max){
            max = *(daclProfile+i);
            daclValue = i;
            max_detect = 1;
        }
    }

    if(max_detect == 0) daclValue = 31;  

    ret = daclValue*8 + 1;
    return ret;
}

// ---------------------------------------------------------------------
// function to analyze dacl profile for Beam calibration
unsigned imxpad_processDaclProfileBEAM(unsigned *daclProfile,unsigned int maxSCurve){
 /*   int daclVal = 0;
    int i;
    unsigned ret = 0;
    unsigned daclValue = 0;
    int diff_cur = 0;
    int diff_max = 0;
    
    int derive1[64];
    int derive2[64];
    
    for(i=0;i<64;i++){
		derive1[i] = 0;
		derive2[i] = 0;
		if(daclProfile[i] < 50)
			daclProfile[i] = 0;
	}
    for (daclVal=0; daclVal<64; daclVal++){
        if (daclVal > 0 && daclVal < 63)
            derive1[daclVal] = (*(daclProfile+daclVal+1))-(*(daclProfile+daclVal-1));
    }    
    for (daclVal=2; daclVal<61; daclVal++){
        if (daclVal > 0 && daclVal < 63)
            derive2[daclVal] = derive1[daclVal+1] - derive1[daclVal-1];
    }
    for (daclVal=0; daclVal<64; daclVal++){
			if(derive2[daclVal] < -1){
				daclValue = daclVal;
				break;
			}
    }
    ret = (daclValue << 3) |  0x0001;
    return ret;
    */
    int max=0;
    int min_derive=0;
    int max_derive=0;
    // generate some data:

    int val;

    int derive[64];
    int derive2[64];
    
    int firstval=0;
    int flag=0;
    int img[64];

    int maxVal = maxSCurve;
        
    for(int ii=0;ii<64;ii++)
        img[ii]=daclProfile[ii];

    for (int i=0; i<64; ++i)
    {
        if(img[i] <= 50)
            img[i] = 0;

        if(daclProfile[i] > 5 && flag == 0){
            flag = 1;
            if(i>4)
            firstval = i-4;
            else firstval = 0;
        }

        if(img[i] > max)
            max = img[i];


        if(i==0)derive[i] = 0;
        else if(i==63) derive[i] = 0;
        else{
            derive[i] = 0;
            derive[i] = img[i+1] - img[i-1];
        }

        if(derive[i] < min_derive)
            min_derive = derive[i];
        if(derive[i] > max_derive)
            max_derive = derive[i];
    }
    derive2[0]=0;
    derive2[1]=0;
    derive2[62]=0;
    derive2[63]=0;
    for(int i=2;i<62;i++){
        derive2[i]=0;
        derive2[i] = derive[i+1]-derive[i-1];
    }
    val = 0;
    for(int dacl=0;dacl<63;dacl++){
        if(derive2[dacl]<-10){
            val = dacl-1;
            break;
        }
    }

    if(derive[val] > maxVal || derive[val+1] > maxVal || derive[val+1] < -maxVal || derive[val] < -maxVal){
        for(int j = 0;j<15;j++){
            if(firstval > 1){
                if(derive[firstval] != 0){
                    val= firstval;
                    firstval--;
                }
                else{
                    val= (firstval-2);
                    break;
                }
            }
            else{
                val = 0;
                break;
            }

        }
    }
    return val;
}

// ---------------------------------------------------------------------
// function to detect noisy pixels and to modify its threshold
unsigned imxpad_processOTNiteration(unsigned modMask, unsigned *daclMatrix, uint16_t *image){
    int i = 0;
    int modNb          = xpci_getModNb(modMask);
    int lastMod        = xpci_getLastMod(modMask);
    int size           = lastMod*120*560;
    int noisyPixels    = 0;
    unsigned daclValue = 0;
    unsigned blockedpixel=0;


    // find noisy pixels
    for (i=0; i<size; i++){
	   if(xpci_getAbortProcess())
            return 1;
        daclValue = daclMatrix[i];
        daclValue = daclValue - 1;
        daclValue = daclValue / 8 ;
        if (image[i]>0){
            noisyPixels++; // increment counter of noisy pixels
            if (daclValue>0){ // decrement DACL if possible
                daclValue = daclValue - 1;
            }
            else blockedpixel++;

        }
        daclMatrix[i] = daclValue * 8 + 1;
    }
    printf("blocked pixels = %d \n",blockedpixel);
    return noisyPixels;
}


// function to detect noisy pixels and to modify its threshold
unsigned imxpad_desableNoisyPixels(unsigned modMask, unsigned *daclMatrix, uint16_t *image){
    int i = 0;
    int modNb          = xpci_getModNb(modMask);
    int lastMod        = xpci_getLastMod(modMask);
    int size           = lastMod*120*560;
    int noisyPixels    = 0;
    unsigned daclValue = 0;

    // find noisy pixels
    for (i=0; i<size; i++){
        if (*(image+i)>4){
            noisyPixels++; // increment counter of noisy pixels
            *(daclMatrix+i) = 0x0004;
        }
    }
    return noisyPixels;
}




// ---------------------------------------------------------------------
// function to make an ithl scan
// all images are read in 16 bits format
int imxpad_scanITHL(unsigned modMask, unsigned Texp, unsigned ithl_min, unsigned ithl_max, char *path){

    //printf("%s() \n", __func__);

    int pos = strlen(path);
    struct stat status;
    int loop;
    char fname[pos+15];
    FILE *wfile;
    unsigned ithlVal = 0;
    uint16_t **ithlImg;
    uint16_t *img;
    int modNb = xpci_getLastMod(modMask);
    int row, col = 0;
    unsigned ovf_time = 4000;

   // if(xpci_systemType == IMXPAD_S1400)
	//ovf_time = 0xFFFFFFFF;

    // allocate buffer for ithl image
    ithlImg = malloc(sizeof(uint16_t *));
    ithlImg[0]=malloc(120*560*modNb*sizeof(uint16_t));

    // check if path is not an empty string
    if(pos == 0){
        printf("%s() ERROR: ITHL scan directory path cannot be an empty string.\n", __func__);
        free(ithlImg[0]);
        return -1;
    }

    // remove '/' char from the end of the string if exist
    while(path[pos-1]=='/'){
        path[pos-1]=NULL;
        pos = strlen(path);
    }

    // check the path and create a directory
    if(stat(path, &status)==0){
        if(S_ISREG(status.st_mode)){
            printf("%s() ERROR: %s is an existing file.\n", __func__, path);
            free(ithlImg[0]);
            return -1;
        }
        if(S_ISDIR(status.st_mode))
            printf("%s() ERROR: %s is an existing folder. Possible file overwriting.\n", __func__, path);
    }
    else{
        if(mkdir(path, S_IRWXU |  S_IRWXG |  S_IRWXO)!=0){
            printf("%s.\n", strerror(errno));
            free(ithlImg[0]);
            return -1;
        }
    }
	xpci_modGlobalAskReady(modMask);
    // configure exposure parameters (images erad in 16 bits format)
    if (xpci_modExposureParam_internal(modMask, Texp, 5000, 0, 0, ovf_time, 0, 0, 0, 1, 3, 0, 0, 0, 0, 0, 0)!= 0)
        printf("%s() ERROR: failed to send exposure parameters\n", __func__);

    // upload DACL = 32 (32*8+1=257)
    if(xpci_modLoadFlatConfig(modMask, 0x7f, 257)!=0){
        printf("%s() ERROR: failed to send flat config 257 (DACL=32). ITHL scan aborted.\n", __func__);
        free(ithlImg[0]);
        return -1;
    }

    // start scan
    for(ithlVal=ithl_min; ithlVal<(ithl_max+1); ithlVal++){

		int ret;
        if(xpci_getAbortProcess())
            return 1;

        printf("\nScan ITHL %d/%d steps\n\n",ithlVal-ithl_min,ithl_max-ithl_min);
        fflush(stdout);
		xpci_modGlobalAskReady(modMask);
        // upload config G
        if(xpci_modLoadConfigG(modMask, 0x7f, ITHL, ithlVal) != 0){
            printf("%s() ERROR: failed to write global register (ITHL=%d)\n", __func__, ithlVal);
            free(ithlImg[0]);
            return -1;
        }

		xpci_modGlobalAskReady(modMask);
    // configure exposure parameters (images erad in 16 bits format)
		if (xpci_modExposureParam_internal(modMask, Texp, 5000, 0, 0, ovf_time, 0, 0, 0, 1, 3, 0, 0, 0, 0, 0, 0)!= 0){
				printf("%s() ERROR: failed to send exposure parameters\n", __func__);
				xpci_modGlobalAskReady(modMask);
		}

		ret = xpci_getImgSeq(B2, modMask, 7, 1, (void *)ithlImg, 0, 0, 0, 0);
        // expose and read
        if(ret!=0){
            printf("%s() ERROR: failed to acquire an image. ITHL scan aborted.\n", __func__);
            free(ithlImg[0]);
            if(ret == 1)
				xpci_setAbortProcess();
            return ret;
        }

        //assign pointer
        sprintf(fname, "%s/ITHL_%d.dat", path,ithlVal);
        wfile = fopen(fname, "w");
        if (wfile==NULL){
            printf("%s() ERROR: failed to open file %s\n", __func__, fname);
            free(ithlImg[0]);
            return -1;
        }

        img = *ithlImg;
        // write data to .dat file
        for (row=0; row<(modNb*120); row++){ // lines
            for (col=0; col<560; col++) // words
                if(col<559){
                    fprintf(wfile, "%"SCNu16" ", *(img+row*560+col));
				}
                else
                    fprintf(wfile, "%"SCNu16, *(img+row*560+col));
            if(row<(modNb*120-1))
                fprintf(wfile, "\n");
        }//
        fclose(wfile);
    }
    printf("\n");
    fflush(stdout);

    free(ithlImg);
    return 0;
}

// ---------------------------------------------------------------------
// function to process ithl scan data
// find the ithl value for which most of the pixels are counting
int imxpad_processIthlScanDataBEAM(unsigned modMask, char *dirpath, unsigned ithl_min, unsigned ithl_max, unsigned *ithlval){

    //printf("%s() \n", __func__);

    FILE *rdfile;
   
    int pos = strlen(dirpath);
    char fname[pos+25];
    struct stat path_status;
    int modNb = xpci_getModNb(modMask);
    unsigned *ithlimg; // current ithl image
    unsigned *buffer; //buffer containing all ithl images
    
    int *derive1;
    int *derive2;
    
    int i, j, row, column;
    unsigned ithlVal,index;
    
    int LineNumber = 120*modNb;
    int ColumnNumber = 560;
    
    int scansize = ithl_max - ithl_min + 1;
    
    // check that directory exist
    if(stat(dirpath, &path_status)==0){
        if(!S_ISDIR(path_status.st_mode)){
            printf("%s() ERROR: %s is not a directory.\n", __func__, dirpath);
            return -1;
        }
    }
    else{
        printf("%s() ERROR: %s directory does not exist.\n", __func__, dirpath);
        return -1;
    }

    //allocate memory
    ithlimg = malloc(LineNumber*ColumnNumber*sizeof(unsigned)); // one image buffer
    buffer = malloc(LineNumber*ColumnNumber*scansize*sizeof(unsigned));//all images buffer
    
    derive1 =(int *) malloc(scansize*sizeof(int));
    derive2 =(int *) malloc(scansize*sizeof(int));

    // open ithl scan files and store them in a global buffer containing all images
    for(ithlVal=ithl_max; ithlVal>=ithl_min; ithlVal--){

        if(xpci_getAbortProcess())
            return 1;

       // index = ithlVal - ithl_min;
       index = ithl_max - ithlVal;
        // build name string
        sprintf(fname, "%s/ITHL_%d.dat", dirpath, ithlVal);
        // open file for reading
        if ((rdfile=fopen(fname, "r"))==NULL){
            printf("%s() ERROR: failed to open file %s.\n", __func__, fname);
            free(ithlimg);
            free(buffer);
            return -1;
        }
        // read file matrix
        printf(".");
        fflush(stdout);
        if(imxpad_readDataMatrix(rdfile, modMask, ithlimg)!=0){
            printf("%s ERROR: failed to read file %s.\n", __func__, fname);
            fclose(rdfile);
            free(ithlimg);
            free(buffer);
            return -1;
        }
        
        fclose(rdfile);

        //Scan each image and count the pixels with noise
        for (row=0;row< LineNumber;row++) {
            for (column=0;column<ColumnNumber;column++){
                //Writting a buffer containting the 64 read images
                if(ithlimg[row*ColumnNumber+column] < 50)
					buffer[index*LineNumber*ColumnNumber + (row*ColumnNumber+column)] = 0;
                else
					buffer[index*LineNumber*ColumnNumber + (row*ColumnNumber+column)] = ithlimg[row*ColumnNumber+column];
                //if (row == 60 && column == 60)
                //    printf("%u ", ithlimg[row*ColumnNumber+column]);
            }
        }
    }

    printf("\n");
    for(i=0;i<scansize;i++){
		derive1[i] = 0;
		derive2[i] = 0;
	}  
    
    int diff_cur=0, diff_max=0;
    unsigned ithlValue;
    for (row=0;row<LineNumber;row++) {
        for (column=0;column<ColumnNumber;column++){

			for(i=0;i<scansize;i++){
				derive1[i] = 0;
				derive2[i] = 0;
			}
    
            if(xpci_getAbortProcess())
                return 1;

            ithlValue=ithl_min;
            diff_cur = 0;
            diff_max = 0;

            //Detect the noise peak (only count above 5 in order not to be affected by cosmics)
         //   for (ithlVal=ithl_max; ithlVal>=ithl_min; ithlVal--)
            for (i=1; i<scansize-1; i++){
                if(xpci_getAbortProcess())
                    return 1;
                    derive1[i] = buffer[(i+1)*LineNumber*ColumnNumber + (row*(ColumnNumber)+column)] - buffer[(i-1)*LineNumber*ColumnNumber + (row*(ColumnNumber)+column)];
            }
			for(i=1;i<scansize-1;i++){
				derive2[i] = derive1[ i + 1] - derive1[i - 1];
			}
            for(i=0;i<scansize;i++){
				if(derive2[i] < -10){
					ithlValue = ithl_max - i;
					break;
				}
			}                       
            ithlimg[(row*(ColumnNumber)+column)] = ithlValue;
            //if (row == 60 && column == 60)
        }
    }
    free(buffer);
    free(derive1);
    free(derive2);
    
    unsigned sum=0, count=0;
    float mean;
    

    //Mean Value of ITHL for each chip of every module
    
    for (i=0; i<modNb; i++){
		printf("\n");
        for (j=0; j<7; j++){ //chipNumber
            if(xpci_getAbortProcess())
                return 1;

            //cout << "chip= " << j << endl;
            for (row=i*120; row<(i+1)*120;row++){
                for (column=j*80; column<(j+1)*80; column++){
                    //cout << "chip= " << j << " row= " << row << " column= " << column << endl;
                    
                    if (ithlimg[row*(ColumnNumber)+column]>=ithl_min){
                        sum += ithlimg[row*(ColumnNumber)+column];
                        count++;
                    }
                }
                
            }
			
            if (count > 0)
                mean = sum / count;
            else
                mean = 0;

            ithlval[i*7 + j] = (unsigned)mean;
            printf("%u ", ithlval[i*7 + j]);
            fflush(stdout);            
            sum = 0;
            count = 0;
        }
	}
	// Added by fred after ALBA
    sprintf(fname, "%s/ITHL_Matrix.dat",dirpath);
    FILE * fd_ithlMatrix;
    fd_ithlMatrix = fopen(fname,"w+");
    if(fd_ithlMatrix == NULL)
		printf("xpci_lib =>>> Can not open file < %s >\n",fname);
	else{
		for(i=0;i<modNb;i++)
		{
			for(row=0;row<120;row++){
				for(column=0;column<560;column++){
					fprintf(fd_ithlMatrix,"%d ",ithlimg[i*560*120+row*560+column]);
				}	// for col
				fprintf(fd_ithlMatrix,"\n");
			}	// for row		
		}// mod
		fclose(fd_ithlMatrix);
	}
		
	printf("\n");
    fflush(stdout);    
   
 //   free (ithlimg); 
    return 0;
}

// ---------------------------------------------------------------------
// function to process ithl scan data
// find the ithl value for which most of the pixels are counting
int imxpad_processIthlScanData(unsigned modMask, char *dirpath, unsigned ithl_min, unsigned ithl_max, unsigned *ithlval){
    FILE *rdfile;
    int pos = strlen(dirpath);
    char fname[pos+15];
    struct stat path_status;
    unsigned i,j =0;
    int modNb = xpci_getModNb(modMask);
    int lastMod = xpci_getLastMod(modMask);
    int firstMod = xpci_getFirstMod(modMask);


    unsigned *ithlimg; // current ithl image
    unsigned *ithlnoisematrix; // accumulation image of the pixels with noise peak detected (if noise detected pixel tagged with 1)
    int scansize = ithl_max-ithl_min+1;
    unsigned *ithlsum; // sum of the pixels with the noise peak detected (per chip)
    int mod, row, chip, col =0;
    
    // check that directory exist
    if(stat(dirpath, &path_status)==0){
        if(!S_ISDIR(path_status.st_mode)){
            printf("%s() ERROR: %s is not a directory.\n", __func__, dirpath);
            return -1;
        }
    }
    else{
        printf("%s() ERROR: %s directory does not exist.\n", __func__, dirpath);
        return -1;
    }

    //allocate memory
    ithlimg = malloc(120*560*lastMod*sizeof(unsigned)); // one image buffer
    ithlnoisematrix = malloc(120*560*lastMod*sizeof(unsigned)); // one image buffer
    ithlsum = malloc(lastMod*7*sizeof(unsigned)); // sum of pixels with the noise peak detected

    // init
    memset(ithlnoisematrix, 0,120*560*lastMod*sizeof(unsigned));
    memset(ithlsum, 0, lastMod*7*sizeof(unsigned));

    // open ithl scan files
    for(i=0; i<scansize; i++){

        if(xpci_getAbortProcess())
            return 1;

        // build name string
        sprintf(fname, "%s/ITHL_%d.dat", dirpath, ithl_max-i);
        // open file for reading
        if ((rdfile=fopen(fname, "r"))==NULL){
            printf("%s() ERROR: failed to open file %s.\n", __func__, fname);
            free(ithlimg);
            free(ithlnoisematrix);
            free(ithlsum);
            return -1;
        }
        // read file matrix
        printf(".");
        fflush(stdout);
        if(imxpad_readDataMatrix(rdfile, modMask, ithlimg)!=0){
            printf("%s ERROR: failed to read file %s.\n", __func__, fname);
            fclose(rdfile);
            free(ithlimg);
            free(ithlnoisematrix);
            free(ithlsum);
            return -1;
        }
        //   else{

        imxpad_searchIthlValues_noise(modMask, ithlnoisematrix, ithlimg);


        for(mod=firstMod; mod<lastMod; mod++){
			if(xpci_getAbortProcess()) return 1;
            if((1 << mod) & modMask)
            {
                *(ithlsum+mod*7)=0;
                *(ithlsum+mod*7+1)=0;
                *(ithlsum+mod*7+2)=0;
                *(ithlsum+mod*7+3)=0;
                *(ithlsum+mod*7+4)=0;
                *(ithlsum+mod*7+5)=0;
                *(ithlsum+mod*7+6)=0;
                for(row=0; row<120; row++){
                    for(chip=0; chip<7; chip++){
                        for(col=0; col<80; col++){
                            if(*(ithlnoisematrix+mod*120*560+row*560+chip*80+col)==1)
                                *(ithlsum+mod*7+chip) += 1;
                        }
                    }
                }
            }

        }

        for(j=firstMod;j<lastMod*7;j++){
            if(*(ithlval+j)==0)
                if(*(ithlsum+j) >= 4800)
                    *(ithlval+j) = ithl_max-i+1;
        }
        fclose(rdfile);
    }//scansize
	printf("\n");
    free(ithlnoisematrix);
    free(ithlsum);
    // free(ithlimg);
    return 0;
}

// function searches a noise peak for every pixel
//  *ithl_noisematrix - a module matrix to record for which pixels the noise peak has been detected
//                      value 0 means that the noise peak for this pixel has not been yet detected 
//                      value 1 means that the noise peak for this pixel has been detected
//  *ithlimg - an ithl scan image
int imxpad_searchIthlValues_noise(unsigned modMask, unsigned *ithl_noisematrix, unsigned *ithlimg){
    int mod, row, chip, col =0;
    // int modNb = xpci_getModNb(modMask);
    int lastMod = xpci_getLastMod(modMask);
    int firstMod = xpci_getFirstMod(modMask);

    for(mod=firstMod; mod<lastMod; mod++){
        if(modMask & (1<<mod) == 0)
            continue;
        for(row=0; row<120; row++){
            for(chip=0; chip<7; chip++){
                for(col=0; col<80; col++){
                    if(*(ithlimg+mod*120*560+row*560+chip*80+col)>10)
                        *(ithl_noisematrix+mod*120*560+row*560+chip*80+col)= 1;
                }
            }
        }
    }
}

// ---------------------------------------------------------------------
//  The OTN (Over-The-Noise) calibration finds a threshold for every pixels just 
//  above the electronic noise for every pixel. 
// 
//  The OTN SLOW is a calibration with reduced electronic noise in cost of the speed
//  of the preamplifier.
// 
//  The process of OTN calibration is compsoed of steps:
//  1. uploading predefined values of the global registers
//  2. perform DACL scan 
//  3. process DACL scan data to obtain DACL matrix
//  4. upload DACL matrix file 
//  5. perform iterations in order to detect noisy pixels and 
//     modify threshold of those pixels (take image, find counting 
//     pixels, modify their threshold, upload new DACL matrix)
//  6. read back all global registers and save them in the file
//
int imxpad_calibration_OTN_slow(unsigned modMask, char *path, unsigned iterations){
    int modNb = xpci_getModNb(modMask);
    int lastMod = xpci_getLastMod(modMask);
    int pos = strlen(path);
    struct stat status;
    char daclscan_path[pos+10];
    char ithlscan_path[pos+10];
    char configg_path[pos+10];
    unsigned *daclmatrix=malloc(120*560*modNb*sizeof(unsigned));
    uint16_t **img = malloc(sizeof(uint16_t *));
    unsigned *ithlval;
    unsigned noisyPixels = 0;
    int i, j = 0;

    // allocate buffer for one image
    img[0]=malloc(120*560*lastMod*sizeof(uint16_t));
    ithlval = malloc(modNb*7*sizeof(unsigned));
    memset(ithlval, 0, modNb*7*sizeof(unsigned));

    // check if path is not an empty string
    if(pos == 0){
        printf("%s() ERROR: DACL scan directory path cannot be an empty string.\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }

    // remove '/' char from the end of the string if exist
    while(path[pos-1]=='/'){
        path[pos-1]=NULL;
        pos = strlen(path);
    }

    // check the path and create a directory
    if(stat(path, &status)==0){
        if(S_ISREG(status.st_mode)){
            printf("%s() ERROR: %s is an existing file.\n", __func__, path);
            free(img);
            free(ithlval);
            return -1;
        }
        if(S_ISDIR(status.st_mode)){
            printf("%s() ERROR: %s is an existing folder.\n", __func__, path);
            free(img);
            free(ithlval);
            return -1;
        }
    }
    else{
        if(mkdir(path, S_IRWXU |  S_IRWXG |  S_IRWXO)!=0){
            printf("%s() ERROR: cannot create directory %s\n", __func__, path);
            free(img);
            free(ithlval);
            return -1;
        }
    }

    // *******************************************************
    // Upload global configuration for slow noise calibration
    printf("\n\nStep 1. Configuring global registers.\n");
    if(xpci_modLoadConfigG(modMask, 0x7f, CMOS_DSBL, 0) != 0){
        printf("%s() ERROR: writing global configuration (CMOS_DSB)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, AMP_TP, 0) != 0){
        printf("%s() ERROR: writing global configuration (AMP_T)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, VADJ, 0) != 0){
        printf("%s() ERROR: writing global configuration (VADJ)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, VREF, 0) != 0){
        printf("%s() ERROR: writing global configuration (VREF)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, IMFP, 5) != 0){
        printf("%s() ERROR: writing global configuration (IMFP)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, IOTA, 40) != 0){
        printf("%s() ERROR: writing global configuration (IOTA)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, IPRE, 60) != 0){
        printf("%s() ERROR: writing global configuration (IPRE)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, ITHL, 30) != 0){
        printf("%s() ERROR: writing global configuration (ITHL)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, ITUNE, 100) != 0){
        printf("%s() ERROR: writing global configuration (ITUNE)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, IBUFFER, 0) != 0){
        printf("%s() ERROR: writing global configuration (IBUFFER)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }


    // ITHL scan
    printf("\n\nStep 2. ITHL scan.\n");
    // build path
    sprintf(ithlscan_path, "%s/ITHL_scan", path);
    if(imxpad_scanITHL(modMask, 1000000, 20, 50, ithlscan_path)!=0){
        printf("%s() ERROR: failed to make an ITHL scan\n", __func__);
        free(img);
        free(ithlval);
        xpci_clearAbortProcess();
        return -1;
    }
    
    if(imxpad_processIthlScanDataOTN(modMask, ithlscan_path, 20, 50, ithlval)!=0){
        printf("%s() ERROR: failed to make an ITHL scan\n", __func__);
        free(img);
        free(ithlval);
        xpci_clearAbortProcess();
        return -1;
    }

    // upload ITHL values
    for(i=0;i<modNb;i++){
        for(j=0;j<7;j++){
            if(xpci_modLoadConfigG(0x01<<i, 0x01<<j, ITHL, *(ithlval+i*7+j)) != 0){
                printf("%s() ERROR: writing global configuration (ITHL)\n", __func__);
                free(img);
                free(ithlval);
                return -1;
            }
        }
    }

    // DACL scan
    printf("\n\nStep 3. DACL scan.\n");
    // build path
    sprintf(daclscan_path, "%s/DACL_scan", path);
    if(imxpad_scanDACL(modMask, 1000000, daclscan_path)!=0){
        printf("%s() ERROR: failed to make a DACL scan\n", __func__);
        free(img);
        free(ithlval);
        xpci_clearAbortProcess();
        return -1;
    }

    // process DACL scan data
    if(imxpad_processDaclScanData(CALIB_OTN, modMask, daclscan_path, daclmatrix,0)!=0){
        printf("%s() ERROR: failed to process DACL scan data\n", __func__);
        free(img);
        free(ithlval);
        xpci_clearAbortProcess();
        return -1;
    }

    // upload initial DACL matrix
    printf("\n\nStep 4. Uploading initial DACL matrix.\n");
    if(imxpad_uploadDaclMatrix(modMask, daclmatrix)!=0){
        printf("%s() ERROR: failed to upload DACL matrix to the detector\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }

    // increment ITHL before adjustement
    imxpad_incrITHL(modMask);

    printf("\n\nStep 6. Adjusting calibration in %d iterations.\n", iterations);
    // configure exposure parameters (images erad in 16 bits format)
    if (xpci_modExposureParam_internal(modMask, 2000000, 5000, 0, 0, 4000, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0)!= 0)
        printf("%s() ERROR: failed to send exposure parameters\n", __func__);
    // iterations
    for(i=0; i<iterations; i++){
        // upload dacl matrix (not for the first iteration since it has been already done)
        if(i!=0){
            if(imxpad_uploadDaclMatrix(modMask, daclmatrix)!=0){
                printf("%s() ERROR: failed to upload DACL matrix to the detector\n", __func__);
                free(img);
                free(ithlval);
                return -1;
            }
        }

        // acquire an image an image
        if(xpci_getImgSeq(B2, modMask, 7, 1, (void *)img, 0, 0, 0, 0)!=0){
            printf("%s() ERROR: failed to acquire an image\n");
            free(img);
            free(ithlval);
            return -1;
        }

        // find counting pixels and modify their threshold
        noisyPixels=imxpad_processOTNiteration(modMask, daclmatrix, img[0]);
        printf("\t%d: found %d noisy pixels\n", i, noisyPixels);
        if(noisyPixels==0){
            printf("Finished calibration optimization, did not find anu noisy pixels after %d iterations\n", i);
            break;
        }
    }

    // save DACL matrix to the file (from memory)
    if(imxpad_saveDaclMatrix(modMask, path, daclmatrix)!=0)
        printf("%s() ERROR: failed to create DACL_matrix.dat file\n", __func__);


    // increment ITHL before registering configg file
    imxpad_incrITHL(modMask);

    // register globl register data
    // create path to dacl_scan directory
    printf("\n\nStep 7. Creating file with global configuration.\n");
    sprintf(configg_path, "%s/configg.cfg", path);
    if(imxpad_fileCreateConfigG(configg_path, modMask)!=0){
        printf("%s() ERROR: faile to fiel with global registers configuration\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }


    free(img);
    free(ithlval);
    
    return 0;
}

int imxpad_calibration_OTN_medium(unsigned modMask, char *path, unsigned iterations){
    int modNb = xpci_getModNb(modMask);
    int pos = strlen(path);
    struct stat status;
    char daclscan_path[pos+10];
    char ithlscan_path[pos+10];
    char configg_path[pos+10];
    unsigned *daclmatrix=malloc(120*560*modNb*sizeof(unsigned));
    uint16_t **img = malloc(sizeof(uint16_t *));
    unsigned *ithlval;
    unsigned noisyPixels = 0;
    int i, j = 0;

    // allocate buffer for one image
    img[0]=malloc(120*560*modNb*sizeof(uint16_t));
    ithlval = malloc(modNb*7*sizeof(unsigned));
    memset(ithlval, 0, modNb*7*sizeof(unsigned));

    // check if path is not an empty string
    if(pos == 0){
        printf("%s() ERROR: DACL scan directory path cannot be an empty string.\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }

    // remove '/' char from the end of the string if exist
    while(path[pos-1]=='/'){
        path[pos-1]=NULL;
        pos = strlen(path);
    }

    // check the path and create a directory
    if(stat(path, &status)==0){
        if(S_ISREG(status.st_mode)){
            printf("%s() ERROR: %s is an existing file.\n", __func__, path);
            free(img);
            free(ithlval);
            return -1;
        }
        if(S_ISDIR(status.st_mode)){
            printf("%s() ERROR: %s is an existing folder.\n", __func__, path);
            free(img);
            free(ithlval);
            return -1;
        }
    }
    else{
        if(mkdir(path, S_IRWXU |  S_IRWXG |  S_IRWXO)!=0){
            printf("%s() ERROR: cannot create directory %s\n", __func__, path);
            free(img);
            free(ithlval);
            return -1;
        }
    }

    // *******************************************************
    // Upload global configuration for slow noise calibration
    printf("\n\nStep 1. Configuring global registers.\n");
    if(xpci_modLoadConfigG(modMask, 0x7f, CMOS_DSBL, 0) != 0){
        printf("%s() ERROR: writing global configuration (CMOS_DSB)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, AMP_TP, 0) != 0){
        printf("%s() ERROR: writing global configuration (AMP_T)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, VADJ, 0) != 0){
        printf("%s() ERROR: writing global configuration (VADJ)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, VREF, 0) != 0){
        printf("%s() ERROR: writing global configuration (VREF)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, IMFP, 25) != 0){
        printf("%s() ERROR: writing global configuration (IMFP)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, IOTA, 40) != 0){
        printf("%s() ERROR: writing global configuration (IOTA)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, IPRE, 60) != 0){
        printf("%s() ERROR: writing global configuration (IPRE)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, ITHL, 30) != 0){
        printf("%s() ERROR: writing global configuration (ITHL)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, ITUNE, 120) != 0){
        printf("%s() ERROR: writing global configuration (ITUNE)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, IBUFFER, 0) != 0){
        printf("%s() ERROR: writing global configuration (IBUFFER)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }


    // ITHL scan
    printf("\n\nStep 2. ITHL scan.\n");
    // build path
    sprintf(ithlscan_path, "%s/ITHL_scan", path);
    if(imxpad_scanITHL(modMask, 1000000, 20, 50, ithlscan_path)!=0){
        printf("%s() ERROR: failed to make an ITHL scan\n", __func__);
        free(img);
        free(ithlval);
        xpci_clearAbortProcess();
        return -1;
    }

    if(imxpad_processIthlScanDataOTN(modMask, ithlscan_path, 20, 50, ithlval)!=0){
        printf("%s() ERROR: failed to make an ITHL scan\n", __func__);
        free(img);
        free(ithlval);
        xpci_clearAbortProcess();
        return -1;
    }

    // upload ITHL values
    for(i=0;i<modNb;i++){
        for(j=0;j<7;j++){
            if(xpci_modLoadConfigG(0x01<<i, 0x01<<j, ITHL, *(ithlval+i*7+j)) != 0){
                printf("%s() ERROR: writing global configuration (ITHL)\n", __func__);
                free(img);
                free(ithlval);
                return -1;
            }
        }
    }

    // DACL scan
    printf("\n\nStep 3. DACL scan.\n");
    // build path
    sprintf(daclscan_path, "%s/DACL_scan", path);
    if(imxpad_scanDACL(modMask, 1000000, daclscan_path)!=0){
        printf("%s() ERROR: failed to make a DACL scan\n", __func__);
        free(img);
        free(ithlval);
        xpci_clearAbortProcess();
        return -1;
    }

    // process DACL scan data
    if(imxpad_processDaclScanData(CALIB_OTN, modMask, daclscan_path, daclmatrix,0)!=0){
        printf("%s() ERROR: failed to process DACL scan data\n", __func__);
        free(img);
        free(ithlval);
        xpci_clearAbortProcess();
        return -1;
    }

    // upload initial DACL matrix
    printf("\n\nStep 4. Uploading initial DACL matrix.\n");
    if(imxpad_uploadDaclMatrix(modMask, daclmatrix)!=0){
        printf("%s() ERROR: failed to upload DACL matrix to the detector\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }

    // increment ITHL before adjustement
    imxpad_incrITHL(modMask);

    printf("\n\nStep 6. Adjusting calibration in %d iterations.\n", iterations);
    // configure exposure parameters (images erad in 16 bits format)
    if (xpci_modExposureParam_internal(modMask, 2000000, 5000, 0, 0, 4000, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0)!= 0)
        printf("%s() ERROR: failed to send exposure parameters\n", __func__);
    // iterations
    for(i=0; i<iterations; i++){
        // upload dacl matrix (not for the first iteration since it has been already done)
        if(i!=0){
            if(imxpad_uploadDaclMatrix(modMask, daclmatrix)!=0){
                printf("%s() ERROR: failed to upload DACL matrix to the detector\n", __func__);
                free(img);
                free(ithlval);
                return -1;
            }
        }

        // acquire an image an image
        if(xpci_getImgSeq(B2, modMask, 7, 1, (void *)img, 0, 0, 0, 0)!=0){
            printf("%s() ERROR: failed to acquire an image\n");
            free(img);
            free(ithlval);
            return -1;
        }

        // find counting pixels and modify their threshold
        noisyPixels=imxpad_processOTNiteration(modMask, daclmatrix, img[0]);
        printf("\t%d: found %d noisy pixels\n", i, noisyPixels);
        if(noisyPixels==0){
            printf("Finished calibration optimization, did not find anu noisy pixels after %d iterations\n", i);
            break;
        }
    }

    // save DACL matrix to the file (from memory)
    if(imxpad_saveDaclMatrix(modMask, path, daclmatrix)!=0)
        printf("%s() ERROR: failed to create DACL_matrix.dat file\n", __func__);


    // increment ITHL before registering configg file
    imxpad_incrITHL(modMask);

    // register globl register data
    // create path to dacl_scan directory
    printf("\n\nStep 7. Creating file with global configuration.\n");
    sprintf(configg_path, "%s/configg.cfg", path);
    if(imxpad_fileCreateConfigG(configg_path, modMask)!=0){
        printf("%s() ERROR: faile to fiel with global registers configuration\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }


    free(img);
    free(ithlval);
    
    return 0;
}

int imxpad_calibration_OTN_fast(unsigned modMask, char *path, unsigned iterations){
    int modNb = xpci_getModNb(modMask);
    int pos = strlen(path);
    struct stat status;
    char daclscan_path[pos+10];
    char ithlscan_path[pos+10];
    char configg_path[pos+10];
    unsigned *daclmatrix=malloc(120*560*modNb*sizeof(unsigned));
    uint16_t **img = malloc(sizeof(uint16_t *));
    unsigned *ithlval;
    unsigned noisyPixels = 0;
    int i, j = 0;

    // allocate buffer for one image
    img[0]=malloc(120*560*modNb*sizeof(uint16_t));
    ithlval = malloc(modNb*7*sizeof(unsigned));
    memset(ithlval, 0, modNb*7*sizeof(unsigned));

    // check if path is not an empty string
    if(pos == 0){
        printf("%s() ERROR: DACL scan directory path cannot be an empty string.\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }

    // remove '/' char from the end of the string if exist
    while(path[pos-1]=='/'){
        path[pos-1]=NULL;
        pos = strlen(path);
    }

    // check the path and create a directory
    if(stat(path, &status)==0){
        if(S_ISREG(status.st_mode)){
            printf("%s() ERROR: %s is an existing file.\n", __func__, path);
            free(img);
            free(ithlval);
            return -1;
        }
        if(S_ISDIR(status.st_mode)){
            printf("%s() ERROR: %s is an existing folder.\n", __func__, path);
            free(img);
            free(ithlval);
            return -1;
        }
    }
    else{
        if(mkdir(path, S_IRWXU |  S_IRWXG |  S_IRWXO)!=0){
            printf("%s() ERROR: cannot create directory %s\n", __func__, path);
            free(img);
            free(ithlval);
            return -1;
        }
    }

    // *******************************************************
    // Upload global configuration for slow noise calibration
    printf("\n\nStep 1. Configuring global registers.\n");
    if(xpci_modLoadConfigG(modMask, 0x7f, CMOS_DSBL, 0) != 0){
        printf("%s() ERROR: writing global configuration (CMOS_DSB)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, AMP_TP, 0) != 0){
        printf("%s() ERROR: writing global configuration (AMP_T)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, VADJ, 0) != 0){
        printf("%s() ERROR: writing global configuration (VADJ)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, VREF, 0) != 0){
        printf("%s() ERROR: writing global configuration (VREF)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, IMFP, 52) != 0){
        printf("%s() ERROR: writing global configuration (IMFP)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, IOTA, 40) != 0){
        printf("%s() ERROR: writing global configuration (IOTA)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, IPRE, 60) != 0){
        printf("%s() ERROR: writing global configuration (IPRE)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, ITHL, 30) != 0){
        printf("%s() ERROR: writing global configuration (ITHL)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, ITUNE, 120) != 0){
        printf("%s() ERROR: writing global configuration (ITUNE)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, IBUFFER, 0) != 0){
        printf("%s() ERROR: writing global configuration (IBUFFER)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }


    // ITHL scan
    printf("\n\nStep 2. ITHL scan.\n");
    // build path
    sprintf(ithlscan_path, "%s/ITHL_scan", path);
    if(imxpad_scanITHL(modMask, 1000000, 20, 50, ithlscan_path)!=0){
        printf("%s() ERROR: failed to make an ITHL scan\n", __func__);
        free(img);
        free(ithlval);
        xpci_clearAbortProcess();
        return -1;
    }
    
    if(imxpad_processIthlScanDataOTN(modMask, ithlscan_path, 20, 50, ithlval)!=0){
        printf("%s() ERROR: failed to make an ITHL scan\n", __func__);
        free(img);
        free(ithlval);
        xpci_clearAbortProcess();
        return -1;
    }

    // upload ITHL values
    for(i=0;i<modNb;i++){
        for(j=0;j<7;j++){
            if(xpci_modLoadConfigG(0x01<<i, 0x01<<j, ITHL, *(ithlval+i*7+j)) != 0){
                printf("%s() ERROR: writing global configuration (ITHL)\n", __func__);
                free(img);
                free(ithlval);
                return -1;
            }
        }
    }

    // DACL scan
    printf("\n\nStep 3. DACL scan.\n");
    // build path
    sprintf(daclscan_path, "%s/DACL_scan", path);
    if(imxpad_scanDACL(modMask, 1000000, daclscan_path)!=0){
        printf("%s() ERROR: failed to make a DACL scan\n", __func__);
        free(img);
        free(ithlval);
        xpci_clearAbortProcess();
        return -1;
    }

    // process DACL scan data
    if(imxpad_processDaclScanData(CALIB_OTN, modMask, daclscan_path, daclmatrix,0)!=0){
        printf("%s() ERROR: failed to process DACL scan data\n", __func__);
        free(img);
        free(ithlval);
        xpci_clearAbortProcess();
        return -1;
    }

    // upload initial DACL matrix
    printf("\n\nStep 4. Uploading initial DACL matrix.\n");
    if(imxpad_uploadDaclMatrix(modMask, daclmatrix)!=0){
        printf("%s() ERROR: failed to upload DACL matrix to the detector\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }

    // increment ITHL before adjustement
    imxpad_incrITHL(modMask);

    printf("\n\nStep 6. Adjusting calibration in %d iterations.\n", iterations);
    // configure exposure parameters (images erad in 16 bits format)
    if (xpci_modExposureParam_internal(modMask, 2000000, 5000, 0, 0, 4000, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0)!= 0)
        printf("%s() ERROR: failed to send exposure parameters\n", __func__);
    // iterations
    for(i=0; i<iterations; i++){
        // upload dacl matrix (not for the first iteration since it has been already done)
        if(i!=0){
            if(imxpad_uploadDaclMatrix(modMask, daclmatrix)!=0){
                printf("%s() ERROR: failed to upload DACL matrix to the detector\n", __func__);
                free(img);
                free(ithlval);
                return -1;
            }
        }

        // acquire an image an image
        if(xpci_getImgSeq(B2, modMask, 7, 1, (void *)img, 0, 0, 0, 0)!=0){
            printf("%s() ERROR: failed to acquire an image\n");
            free(img);
            free(ithlval);
            return -1;
        }

        // find counting pixels and modify their threshold
        noisyPixels=imxpad_processOTNiteration(modMask, daclmatrix, img[0]);
        printf("\t%d: found %d noisy pixels\n", i, noisyPixels);
        if(noisyPixels==0){
            printf("Finished calibration optimization, did not find anu noisy pixels after %d iterations\n", i);
            break;
        }
    }

    // save DACL matrix to the file (from memory)
    if(imxpad_saveDaclMatrix(modMask, path, daclmatrix)!=0)
        printf("%s() ERROR: failed to create DACL_matrix.dat file\n", __func__);


    // increment ITHL before registering configg file
    imxpad_incrITHL(modMask);

    // register globl register data
    // create path to dacl_scan directory
    printf("\n\nStep 7. Creating file with global configuration.\n");
    sprintf(configg_path, "%s/configg.cfg", path);
    if(imxpad_fileCreateConfigG(configg_path, modMask)!=0){
        printf("%s() ERROR: faile to fiel with global registers configuration\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }


    free(img);
    free(ithlval);
    
    return 0;
}

int imxpad_calibration_OTN(unsigned modMask, char *path, unsigned iterations, unsigned itune, unsigned imfp){
    int modNb = xpci_getModNb(modMask);
    int lastMod = xpci_getLastMod(modMask);
    int firstMod = xpci_getFirstMod(modMask);
    int pos = strlen(path);
    struct stat status;
    char daclscan_path[pos+10];
    char ithlscan_path[pos+10];
    char configg_path[pos+10];
    unsigned *daclmatrix=malloc(120*560*lastMod*sizeof(unsigned));
    int ret = 0;
    uint16_t **img = malloc(sizeof(uint16_t *));
    unsigned *ithlval;
    unsigned noisyPixels = 0;
    unsigned m_noisyPixels=0;
     FILE *wfile;
     char *fname[200];
     int row,col;
    int i, j = 0;
    // allocate buffer for one image
    img[0]=malloc(120*560*lastMod*sizeof(uint16_t));
    ithlval = malloc(lastMod*7*sizeof(unsigned));
    memset(ithlval, 0, lastMod*7*sizeof(unsigned));
    
    xpci_clearAbortProcess();

    // check if path is not an empty string
    if(pos == 0){
        printf("%s() ERROR: DACL scan directory path cannot be an empty string.\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }

    // remove '/' char from the end of the string if exist
    while(path[pos-1]=='/'){
        path[pos-1]=NULL;
        pos = strlen(path);
    }

    // check the path and create a directory
    if(stat(path, &status)==0){
        if(S_ISREG(status.st_mode)){
            printf("%s() ERROR: %s is an existing file.\n", __func__, path);
            free(img);
            free(ithlval);
            return -1;
        }
        if(S_ISDIR(status.st_mode)){
            printf("%s() ERROR: %s is an existing folder.\n", __func__, path);
            free(img);

            free(ithlval);
            return -1;
        }
    }
    else{
        if(mkdir(path, S_IRWXU |  S_IRWXG |  S_IRWXO)!=0){
            printf("%s() ERROR: cannot create directory %s\n", __func__, path);
            free(img);
            free(ithlval);
            return -1;
        }
    }
	xpci_modGlobalAskReady(modMask);
    // *******************************************************
    // Upload global configuration for slow noise calibration
    printf("\n\nStep 1. Configuring global registers.\n");
    if(xpci_modLoadConfigG(modMask, 0x7f, CMOS_DSBL, 0) != 0){
        printf("%s() ERROR: writing global configuration (CMOS_DSB)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, AMP_TP, 0) != 0){
        printf("%s() ERROR: writing global configuration (AMP_T)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, VADJ, 0) != 0){
        printf("%s() ERROR: writing global configuration (VADJ)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, VREF, 0) != 0){
        printf("%s() ERROR: writing global configuration (VREF)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, IMFP, imfp) != 0){
        printf("%s() ERROR: writing global configuration (IMFP)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, IOTA, 40) != 0){
        printf("%s() ERROR: writing global configuration (IOTA)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, IPRE, 60) != 0){
        printf("%s() ERROR: writing global configuration (IPRE)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, ITHL, 30) != 0){
        printf("%s() ERROR: writing global configuration (ITHL)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, ITUNE, itune) != 0){
        printf("%s() ERROR: writing global configuration (ITUNE)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, IBUFFER, 0) != 0){
        printf("%s() ERROR: writing global configuration (IBUFFER)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }

    if(xpci_getAbortProcess())
            return 1;
    // ITHL scan
    printf("\n\nStep 2. ITHL scan.\n");
    // build path
    sprintf(ithlscan_path, "%s/ITHL_scan", path);
    xpci_modGlobalAskReady(modMask);
    if(imxpad_scanITHL(modMask, 1000000, 20, 50, ithlscan_path)==-1){
        printf("%s() ERROR: failed to make an ITHL scan\n", __func__);
        free(img);
        free(ithlval);
        xpci_clearAbortProcess();
        return -1;
    }
    if(xpci_getAbortProcess())
            return 1;
    if(imxpad_processIthlScanData(modMask, ithlscan_path, 20, 50, ithlval)==-1){
        printf("%s() ERROR: failed to make an ITHL scan\n", __func__);
        free(img);
        free(ithlval);
        xpci_clearAbortProcess();
        return -1;
    }
    if(xpci_getAbortProcess())
            return 1;    
            
    /*        
    // upload ITHL values
    for(i=firstMod;i<lastMod;i++){
	    if(xpci_getAbortProcess())
            return 1;
        int result=0;
        if(modMask & (1 << i))
        {
            result = 0;
            for(j=0;j<7;j++){
                result += *(ithlval+i*7+j);
            }
            result = result / 7;
            printf("Mask = 0x%x result ITHL = %d\n", (1 << i),result);
            if(xpci_modLoadConfigG(0x01<<i, 0x7F, ITHL, result) ==-1){
                printf("%s() ERROR: writing global configuration (ITHL)\n", __func__);
                free(img);
                free(ithlval);
                return -1;
            }
        }
    }
	*/	
	 for(i=firstMod;i<lastMod;i++){
	    if(xpci_getAbortProcess())
            return 1;
            
        if(modMask & (1 << i))
        {           
		    printf(" Mod Mask = 0x%x ITHL =>",modMask & (1 << i));
            for(j=0;j<7;j++){
                printf(" %d ",*(ithlval+i*7+j));
				if(xpci_modLoadConfigG(0x01<<i, 0x01<<j, ITHL, *(ithlval+i*7+j)) ==-1){
					printf("%s() ERROR: writing global configuration (ITHL)\n", __func__);
					free(img);
					free(ithlval);
					return -1;
				}
			}
			printf("\n");
        }
    }
    // DACL scan
    printf("\n\nStep 3. DACL scan.\n");
    // build path
    sprintf(daclscan_path, "%s/DACL_scan", path);
    if(imxpad_scanDACL(modMask, 1000000, daclscan_path)==-1){
        printf("%s() ERROR: failed to make a DACL scan\n", __func__);
        free(img);
        free(ithlval);
        xpci_clearAbortProcess();
        return -1;
    }
    if(xpci_getAbortProcess())
            return 1;
    // process DACL scan data
    if(imxpad_processDaclScanData(CALIB_OTN, modMask, daclscan_path, daclmatrix,0)==-1){
        printf("%s() ERROR: failed to process DACL scan data\n", __func__);
        free(img);
        free(ithlval);
        xpci_clearAbortProcess();
        return -1;
    }
    if(xpci_getAbortProcess())
            return 1;    
    // increment ITHL before adjustement
    imxpad_incrITHL(modMask);
    if(xpci_getAbortProcess())
            return 1;
    // upload initial DACL matrix
    printf("\n\nStep 4. Uploading initial DACL matrix.\n");
    if(imxpad_uploadDaclMatrix(modMask, daclmatrix)==-1){
        printf("%s() ERROR: failed to upload DACL matrix to the detector\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_getAbortProcess())
            return 1;
    printf("\n\nStep 6. Adjusting calibration in %d iterations.\n", iterations);
    // configure exposure parameters (images erad in 16 bits format)
    if (xpci_modExposureParam_internal(modMask, 2000000, 5000, 0, 0, 4000, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0)!= 0)
        printf("%s() ERROR: failed to send exposure parameters\n", __func__);
    // iterations

    for(i=0; i<iterations; i++){	
		
		if(xpci_getAbortProcess())
            return 1;	
	    // upload dacl matrix (not for the first iteration since it has been already done)
        if(i!=0){
            if(imxpad_uploadDaclMatrix(modMask, daclmatrix)==-1){
                printf("%s() ERROR: failed to upload DACL matrix to the detector\n", __func__);
                free(img);
                free(ithlval);
                return -1;
            }
        }
		if(xpci_getAbortProcess())
            return 1;

        ret = xpci_getImgSeq(B2, modMask, 7, 1, (void *)img, 0, 0, 0, 0);
        // acquire an image an image
        if(ret==-1){
            printf("%s() ERROR: failed to acquire an image\n");
            free(img);
            free(ithlval);
            return -1;
        }
        if(ret == 1)
			return 1;

        // find counting pixels and modify their threshold
        noisyPixels=imxpad_processOTNiteration(modMask, daclmatrix, img[0]);
        printf("\t%d: found %d noisy pixels\n", i, noisyPixels);
        if(noisyPixels==0){
            printf("Finished calibration optimization, did not find anu noisy pixels after %d iterations\n", i);
            break;
        }
        
                //assign pointer
        sprintf(fname, "%s/Step_%d.dat", daclscan_path,i);
        wfile = fopen(fname, "w");
        if (wfile==NULL){
            printf("%s() ERROR: failed to open file %s\n", __func__, fname);
            continue;
        }

        // write data to .dat file
        for (row=0; row<(lastMod*120); row++){ // lines
            for (col=0; col<560; col++) // words
                if(col<559)
                    fprintf(wfile, "%"SCNu16" ", *(img[0]+row*560+col));
                else
                    fprintf(wfile, "%"SCNu16, *(img[0]+row*560+col));
            if(row<(lastMod*120-1))
                fprintf(wfile, "\n");
        }//
        fclose(wfile);
    }
    // increment ITHL before registering configg file

    imxpad_incrITHL(modMask);
    if(xpci_getAbortProcess())
            return 1;
    if(imxpad_uploadDaclMatrix(modMask, daclmatrix)==-1){
        printf("%s() ERROR: failed to upload DACL matrix to the detector\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }

	if(xpci_getAbortProcess())
            return 1;
    ///   ************* end *****************************

    // save DACL matrix to the file (from memory)
    if(imxpad_saveDaclMatrix(modMask, path, daclmatrix)==-1)
        printf("%s() ERROR: failed to create DACL_matrix.dat file\n", __func__);

	if(xpci_getAbortProcess())
            return 1;


    // register globl register data
    // create path to dacl_scan directory
    printf("\n\nStep 7. Creating file with global configuration.\n");
    sprintf(configg_path, "%s/configg.cfg", path);
    printf(configg_path, "%s/configg.cfg", path);fflush(stdout);

    if(imxpad_fileCreateConfigG(configg_path, modMask)==-1){
        printf("%s() ERROR: faile to fiel with global registers configuration\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }

    free(img);
    free(ithlval);
    
    return 0;
}


int imxpad_calibration_BEAM(unsigned modMask, char *path, unsigned Texp, unsigned ithl_max, unsigned itune, unsigned imfp,unsigned int maxSCurve){
    int modNb = xpci_getModNb(modMask);
    int pos = strlen(path);
    struct stat status;
    char daclscan_path[pos+10];
    char ithlscan_path[pos+10];
    char configg_path[pos+10];
    unsigned *daclmatrix;
    uint16_t **img = malloc(sizeof(uint16_t *));
    unsigned *ithlval;
    int i, j = 0;

    xpci_clearAbortProcess();
    // allocate buffer for one image
    img[0]=malloc(120*560*modNb*sizeof(uint16_t));
    // allocate buffer for dacl matrix
    daclmatrix=malloc(120*560*modNb*sizeof(unsigned));
    // allocate buffer for ithl values
    ithlval = malloc(modNb*7*sizeof(unsigned));

    // initialize ithl values with 0
    memset(ithlval, 0, modNb*7*sizeof(unsigned));

    // check if path is not an empty string
    if(pos == 0){
        printf("%s() ERROR: DACL scan directory path cannot be an empty string.\n", __func__);
        free(img);
        free(ithlval);
        free(daclmatrix);
        return -1;
    }

    // remove '/' char from the end of the string if exist
    while(path[pos-1]=='/'){
        path[pos-1]=NULL;
        pos = strlen(path);
    }

    // check the path and create a directory
    if(stat(path, &status)==0){
        if(S_ISREG(status.st_mode)){
            printf("%s() ERROR: %s is an existing file.\n", __func__, path);
            free(img);
            free(ithlval);
            free(daclmatrix);
            return -1;
        }
        if(S_ISDIR(status.st_mode)){
            printf("%s() ERROR: %s is an existing folder.\n", __func__, path);
            free(img);
            free(ithlval);
            free(daclmatrix);
            return -1;
        }
    }
    else{
        if(mkdir(path, S_IRWXU |  S_IRWXG |  S_IRWXO)!=0){
            printf("%s() ERROR: cannot create directory %s\n", __func__, path);
            free(img);
            free(ithlval);
            free(daclmatrix);
            return -1;
        }
    }

     xpci_modGlobalAskReady(modMask);
    // *******************************************************
    // Upload global configuration for slow noise calibration
    printf("\n\nStep 1. Configuring global registers.\n");
    if(xpci_modLoadConfigG(modMask, 0x7f, CMOS_DSBL, 0) ==-1){
        printf("%s() ERROR: writing global configuration (CMOS_DSB)\n", __func__);
        free(img);
        free(ithlval);
        free(daclmatrix);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, AMP_TP, 0) ==-1){
        printf("%s() ERROR: writing global configuration (AMP_T)\n", __func__);
        free(img);
        free(ithlval);
        free(daclmatrix);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, VADJ, 0) ==-1){
        printf("%s() ERROR: writing global configuration (VADJ)\n", __func__);
        free(img);
        free(daclmatrix);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, VREF, 0) ==-1){
        printf("%s() ERROR: writing global configuration (VREF)\n", __func__);
        free(img);
        free(daclmatrix);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, IMFP, imfp) ==-1){
        printf("%s() ERROR: writing global configuration (IMFP)\n", __func__);
        free(img);
        free(ithlval);
        free(daclmatrix);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, IOTA, 40) ==-1){
        printf("%s() ERROR: writing global configuration (IOTA)\n", __func__);
        free(img);
        free(ithlval);
        free(daclmatrix);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, IPRE, 60) ==-1){
        printf("%s() ERROR: writing global configuration (IPRE)\n", __func__);
        free(img);
        free(daclmatrix);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, ITHL, 30) ==-1){
        printf("%s() ERROR: writing global configuration (ITHL)\n", __func__);
        free(img);
        free(daclmatrix);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, ITUNE, itune) ==-1){
        printf("%s() ERROR: writing global configuration (ITUNE)\n", __func__);
        free(img);
        free(ithlval);
        free(daclmatrix);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, IBUFFER, 0) ==-1){
        printf("%s() ERROR: writing global configuration (IBUFFER)\n", __func__);
        free(img);
        free(ithlval);
        free(daclmatrix);
        return -1;
    }


    // ITHL scan
    printf("\n\nStep 2. ITHL scan.\n");
    // build path
    sprintf(ithlscan_path, "%s/ITHL_scan", path);
    if(imxpad_scanITHL(modMask, Texp, 20, ithl_max, ithlscan_path)==-1){
        printf("%s() ERROR: failed to make an ITHL scan\n", __func__);
        free(img);
        free(ithlval);
        free(daclmatrix);
        xpci_clearAbortProcess();
        return -1;
    }
   if(xpci_getAbortProcess())
            return 1;

    if(imxpad_processIthlScanDataBEAM(modMask, ithlscan_path, 20, ithl_max, ithlval)==-1){
        printf("%s() ERROR: failed to make a ITHL scan\n", __func__);
        free(img);
        free(ithlval);
        free(daclmatrix);
        xpci_clearAbortProcess();
        return -1;
    }
   if(xpci_getAbortProcess())
            return 1;
    xpci_modGlobalAskReady(modMask);
    // upload ITHL values
    for(i=0;i<modNb;i++){
        for(j=0;j<7;j++){
            if(xpci_modLoadConfigG(0x01<<i, 0x01<<j, ITHL, *(ithlval+i*7+j)) ==-1){
                printf("%s() ERROR: writing global configuration (ITHL)\n", __func__);
                free(img);
                free(ithlval);
                free(daclmatrix);
                return -1;
            }
        }
    }
   if(xpci_getAbortProcess())
            return 1;
    // DACL scan
    printf("\n\nStep 3. DACL scan.\n");
    // build path
    sprintf(daclscan_path, "%s/DACL_scan", path);
    if(imxpad_scanDACL(modMask, Texp, daclscan_path)==-1){
        printf("%s() ERROR: failed to make a DACL scan\n", __func__);
        free(img);
        free(ithlval);
        free(daclmatrix);
        xpci_clearAbortProcess();
        return -1;
    }
   if(xpci_getAbortProcess())
            return 1;
    // process DACL scan data
    if(imxpad_processDaclScanData(CALIB_BEAM, modMask, daclscan_path, daclmatrix,maxSCurve)==-1){
        printf("%s() ERROR: failed to process DACL scan data\n", __func__);
        free(img);
        free(ithlval);
        free(daclmatrix);
        xpci_clearAbortProcess();
        return -1;
    }
   if(xpci_getAbortProcess())
            return 1;
    // upload DACL matrix
    printf("\n\nStep 4. Uploading initial DACL matrix.\n");
    if(imxpad_uploadDaclMatrix(modMask, daclmatrix)==-1){
        printf("%s() ERROR: failed to upload DACL matrix to the detector\n", __func__);
        free(img);
        free(ithlval);
        free(daclmatrix);
        return -1;
    }
   if(xpci_getAbortProcess())
            return 1;
    // save DACL matrix to the file (from memory)
    if(imxpad_saveDaclMatrix(modMask, path, daclmatrix)==-1)
        printf("%s() ERROR: failed to create DACL_matrix.dat file\n", __func__);

    // register globl register data
    // create path to dacl_scan directory
    sprintf(configg_path, "%s/configg.cfg", path);
    if(imxpad_fileCreateConfigG(configg_path, modMask)==-1){
        printf("%s() ERROR: faile to fiel with global registers configuration\n", __func__);
        free(img);
        free(ithlval);
        free(daclmatrix);
        return -1;
    }
   if(xpci_getAbortProcess())
            return 1;
    free(img);
    free(ithlval);
    free(daclmatrix);
    return 0;
}

// ---------------------------------------------------------------------
int imxpad_uploadCalibration(unsigned modMask, char *path){
    int pos = strlen(path);
    struct stat path_status;
    struct stat dacl_status;
    struct stat cfgg_status;
    char daclmatrix_path[pos+30];
    char configg_path[pos+10];
    
    xpci_clearAbortProcess();
    // check if path is not an empty string
    if(pos == 0){
        printf("%s() ERROR: DACL scan directory path cannot be an empty string.\n", __func__);
        return -1;
    }

    // remove '/' char from the end of the string if exist
    while(path[pos-1]=='/'){
        path[pos-1]=NULL;
        pos = strlen(path);
    }

    // create paths for dacl matrix and config g file
    sprintf(daclmatrix_path, "%s/DACL_matrix.dat", path);
    sprintf(configg_path, "%s/configg.cfg", path);

    //check if files exists
    if(stat(daclmatrix_path, &dacl_status)!=0){
        printf("%s() ERROR: %s does not exist.\n", __func__, daclmatrix_path);
        return -1;
    }
    if(stat(configg_path, &cfgg_status)!=0){
        printf("%s() ERROR: %s does not exist.\n", __func__, configg_path);
        return -1;
    }
	if(xpci_getAbortProcess())	return 1;
    // *******************************************************
    // register globl register data
    // create path to dacl_scan directory
    printf("\n\nStep 1. Uloading global configuration.\n");
    if(imxpad_fileUploadConfigG(configg_path)!=0){
        printf("%s() ERROR: failed to upload global configuration file\n", __func__);
        return -1;
    }
	if(xpci_getAbortProcess())	return 1;
    // *******************************************************
    // register globl register data
    // create path to dacl_scan directory
    printf("\n\nStep 2. Uloading DACL matrix.\n");
    if(imxpad_fileUploadDaclMatrix(daclmatrix_path, modMask)!=0){
        printf("%s() ERROR: failed to upload DACL matrix\n", __func__);
        return -1;
    }

    xpci_clearAbortProcess();
    return 0;
}
















///////////////////////////////////////////////////////////////////////////



int imxpad_calibration_OTN_pulse(unsigned modMask, char *path, unsigned iterations, unsigned itune, unsigned imfp){
    int modNb = xpci_getModNb(modMask);
    int lastMod = xpci_getLastMod(modMask);
    int firstMod = xpci_getFirstMod(modMask);
    int pos = strlen(path);
    struct stat status;
    char daclscan_path[pos+10];
    char ithlscan_path[pos+10];
    char configg_path[pos+10];
    unsigned *daclmatrix=malloc(120*560*lastMod*sizeof(unsigned));

    uint16_t **img = malloc(sizeof(uint16_t *));
    unsigned *ithlval;
    unsigned noisyPixels = 0;
    int i, j = 0;
    // allocate buffer for one image
    img[0]=malloc(120*560*lastMod*sizeof(uint16_t));
    ithlval = malloc(lastMod*7*sizeof(unsigned));
    memset(ithlval, 0, lastMod*7*sizeof(unsigned));
	xpci_clearAbortProcess();
    // check if path is not an empty string
    if(pos == 0){
        printf("%s() ERROR: DACL scan directory path cannot be an empty string.\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }

    // remove '/' char from the end of the string if exist
    while(path[pos-1]=='/'){
        path[pos-1]=NULL;
        pos = strlen(path);
    }

    // check the path and create a directory
    if(stat(path, &status)==0){
        if(S_ISREG(status.st_mode)){
            printf("%s() ERROR: %s is an existing file.\n", __func__, path);
            free(img);
            free(ithlval);
            return -1;
        }
        if(S_ISDIR(status.st_mode)){
            printf("%s() ERROR: %s is an existing folder.\n", __func__, path);
            free(img);

            free(ithlval);
            return -1;
        }
    }
    else{
        if(mkdir(path, S_IRWXU |  S_IRWXG |  S_IRWXO)!=0){
            printf("%s() ERROR: cannot create directory %s\n", __func__, path);
            free(img);
            free(ithlval);
            return -1;
        }
    }

    // *******************************************************
    // Upload global configuration for slow noise calibration
    printf("\n\nStep 1. Configuring global registers.\n");
    if(xpci_modLoadConfigG(modMask, 0x7f, CMOS_DSBL, 0) != 0){
        printf("%s() ERROR: writing global configuration (CMOS_DSB)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, AMP_TP, 0) != 0){
        printf("%s() ERROR: writing global configuration (AMP_T)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, VADJ, 0) != 0){
        printf("%s() ERROR: writing global configuration (VADJ)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, VREF, 0) != 0){
        printf("%s() ERROR: writing global configuration (VREF)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, IMFP, imfp) != 0){
        printf("%s() ERROR: writing global configuration (IMFP)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, IOTA, 40) != 0){
        printf("%s() ERROR: writing global configuration (IOTA)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, IPRE, 60) != 0){
        printf("%s() ERROR: writing global configuration (IPRE)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, ITHL, 30) != 0){
        printf("%s() ERROR: writing global configuration (ITHL)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, ITUNE, itune) != 0){
        printf("%s() ERROR: writing global configuration (ITUNE)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_modLoadConfigG(modMask, 0x7f, IBUFFER, 0) != 0){
        printf("%s() ERROR: writing global configuration (IBUFFER)\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }


    // ITHL scan
    printf("\n\nStep 2. ITHL scan.\n");
    // build path
    sprintf(ithlscan_path, "%s/ITHL_scan", path);
    if(imxpad_scanITHL(modMask, 1000000, 20, 50, ithlscan_path)==-1){
        printf("%s() ERROR: failed to make an ITHL scan\n", __func__);
        free(img);
        free(ithlval);
        xpci_clearAbortProcess();
        return -1;
    }
    if(xpci_getAbortProcess())
            return 1;

    if(imxpad_processIthlScanData(modMask, ithlscan_path, 20, 50, ithlval)==-1){
        printf("%s() ERROR: failed to make an ITHL scan\n", __func__);
        free(img);
        free(ithlval);
        xpci_clearAbortProcess();
        return -1;
    }
        if(xpci_getAbortProcess())
            return 1;
    // upload ITHL values
    printf("\n");
    for(i=firstMod;i<lastMod;i++){
        if(modMask & (1 << i))
        {
            printf(" Mod Mask = 0x%x ITHL =>",modMask & (1 << i));
            for(j=0;j<7;j++){
                printf(" %d",*(ithlval+i*7+j));
                if(xpci_modLoadConfigG(0x01<<i, 0x01<<j, ITHL, *(ithlval+i*7+j)) != 0){
                    printf("%s() ERROR: writing global configuration (ITHL)\n", __func__);
                    free(img);
                    free(ithlval);
                    return -1;
                }
            }
         if(xpci_getAbortProcess())
            return 1;
            printf("\n");
        }
    }

    if(xpci_getAbortProcess())
            return 1;
    // DACL scan
    printf("\n\nStep 3. DACL scan.\n");
    // build path
    sprintf(daclscan_path, "%s/DACL_scan", path);
    if(imxpad_scanDACL_pulse(modMask, 100, daclscan_path)==-1){
        printf("%s() ERROR: failed to make a DACL scan\n", __func__);
        free(img);
        free(ithlval);
        xpci_clearAbortProcess();
        return -1;
    }
    if(xpci_getAbortProcess())
            return 1;
    // process DACL scan data
    if(imxpad_processDaclScanData(CALIB_OTN, modMask, daclscan_path, daclmatrix,0)==-1){
        printf("%s() ERROR: failed to process DACL scan data\n", __func__);
        free(img);
        free(ithlval);
        xpci_clearAbortProcess();
        return -1;
    }
        if(xpci_getAbortProcess())
            return 1;
    // increment ITHL before adjustement
    imxpad_incrITHL(modMask);

    if(xpci_getAbortProcess())
            return 1;
    // upload initial DACL matrix
    printf("\n\nStep 4. Uploading initial DACL matrix.\n");
    if(imxpad_uploadDaclMatrix(modMask, daclmatrix)==-1){
        printf("%s() ERROR: failed to upload DACL matrix to the detector\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }

    printf("\n\nStep 6. Adjusting calibration in %d iterations.\n", iterations);
    if(xpci_getAbortProcess())
            return 1;
    // configure exposure parameters (images erad in 16 bits format)
    if (xpci_modExposureParam_internal(modMask, 2000000, 5000, 0, 0, 4000, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0)!= 0)
        printf("%s() ERROR: failed to send exposure parameters\n", __func__);
    // iterations
    for(i=0; i<iterations; i++){
		int ret;
		if(xpci_getAbortProcess())
            return 1;
        // upload dacl matrix (not for the first iteration since it has been already done)
        if(i!=0){
            if(imxpad_uploadDaclMatrix(modMask, daclmatrix)==-1){
                printf("%s() ERROR: failed to upload DACL matrix to the detector\n", __func__);
                free(img);
                free(ithlval);
                return -1;
            }
        }

		ret = xpci_getImgSeq(B2, modMask, 7, 1, (void *)img, 0, 0, 0, 0);
        // acquire an image an image
        if(ret == -1){
            printf("%s() ERROR: failed to acquire an image\n");
            free(img);
            free(ithlval);
            return -1;
        }
		if(xpci_getAbortProcess() || ret == 1)
            return 1;
        // find counting pixels and modify their threshold
        noisyPixels=imxpad_processOTNiteration(modMask, daclmatrix, img[0]);
        printf("\t%d: found %d noisy pixels\n", i, noisyPixels);
        if(noisyPixels==0){
            printf("Finished calibration optimization, did not find anu noisy pixels after %d iterations\n", i);
            break;
        }

    }
	if(xpci_getAbortProcess())
            return 1;
    // increment ITHL before registering configg file
    imxpad_incrITHL(modMask);
	if(xpci_getAbortProcess())
            return 1;
    if(imxpad_uploadDaclMatrix(modMask, daclmatrix)==-1){
        printf("%s() ERROR: failed to upload DACL matrix to the detector\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }
    if(xpci_getAbortProcess())
            return 1;

    ///   ************* end *****************************

    // save DACL matrix to the file (from memory)
    if(imxpad_saveDaclMatrix(modMask, path, daclmatrix)==-1)
        printf("%s() ERROR: failed to create DACL_matrix.dat file\n", __func__);

	
	if(xpci_getAbortProcess())
            return 1;

    // register globl register data
    // create path to dacl_scan directory
    printf("\n\nStep 7. Creating file with global configuration.\n");
    sprintf(configg_path, "%s/configg.cfg", path);
    printf(configg_path, "%s/configg.cfg", path);fflush(stdout);

    if(imxpad_fileCreateConfigG(configg_path, modMask)!=0){
        printf("%s() ERROR: faile to fiel with global registers configuration\n", __func__);
        free(img);
        free(ithlval);
        return -1;
    }


    free(img);
    free(ithlval);
    
    return 0;
}






// ---------------------------------------------------------------------
// function to make a dacl scan
// all images are read in 16 bits format
int imxpad_scanDACL_pulse(unsigned modMask, unsigned nbPulse, char *path){
    int pos = strlen(path);
    struct stat status;
    unsigned daclVal = 0;
    char fname[pos+15];
    FILE *wfile;
    uint16_t **daclImg;
    uint16_t *img;
    int modNb = xpci_getModNb(modMask);
    int firstMod = xpci_getFirstMod(modMask);
    int lastMod    = xpci_getLastMod(modMask);
    int row, col = 0;

    // allocate buffer for dacl image
    daclImg = malloc(sizeof(uint16_t *));
    daclImg[0]=malloc(120*560*lastMod*sizeof(uint16_t));

    // check if path is not an empty string
    if(pos == 0){
        printf("%s() ERROR: DACL scan directory path cannot be an empty string.\n", __func__);
        free(daclImg[0]);
        return -1;
    }

    // remove '/' char from the end of the string if exist
    while(path[pos-1]=='/'){
        path[pos-1]=NULL;
        pos = strlen(path);
    }

    // check the path and create a directory
    if(stat(path, &status)==0){
        if(S_ISREG(status.st_mode)){
            printf("%s() ERROR: %s is an existing file.\n", __func__, path);
            free(daclImg[0]);
            return -1;
        }
        if(S_ISDIR(status.st_mode))
            printf("%s() ERROR: %s is an existing folder. Possible file overwriting.\n", __func__, path);
    }
    else{
        if(mkdir(path, S_IRWXU |  S_IRWXG |  S_IRWXO)!=0){
            printf("%s.\n", strerror(errno));
            free(daclImg[0]);
            return -1;
        }
    }

    if(xpci_modLoadConfigG(modMask, 0x7f, AMP_TP, 30) != 0){
        printf("%s() ERROR: writing global configuration (AMP_T)\n", __func__);
        free(daclImg[0]);
        return -1;
    }


    // start scan
    for(daclVal=0; daclVal<64; daclVal++){
		int ret;

        if(xpci_getAbortProcess())
            return 1;

        printf(".");
        fflush(stdout);
        // send flat dacl
        if(xpci_modLoadFlatConfig(modMask, 0x7f, daclVal*8+1)!=0){
            printf("%s() ERROR: failed to send flat config %d (DACL=%d). DACL scan aborted.\n", __func__, daclVal*8+1, daclVal);
            free(daclImg[0]);
            return -1;
        }
        /////////////////

        printf("sending test pulse (DACL = %d )\n",daclVal);
        if(xpci_pulserImxpad(modMask,nbPulse, 0)!=0){
            printf("%s() ERROR: failed to send pulse %d (DACL=%d). DACL scan aborted.\n", __func__, daclVal*8+1, daclVal);
            free(daclImg[0]);
            return -1;   
          }
       if(xpci_getAbortProcess())
            return 1;

        printf("read image \n");
        ret = xpci_readOneImage(B2, modMask, 7, daclImg[0]);
        if(ret == -1)
            return -1;
         else if (ret == 1)
			return 1;

        //////////////////

        //assign pointer
        sprintf(fname, "%s/DACL_%d.dat", path,daclVal);
        wfile = fopen(fname, "w");
        if (wfile==NULL){
            printf("%s() ERROR: failed to open file %s\n", __func__, fname);
            free(daclImg[0]);
            return -1;
        }

        img = *daclImg;
        // write data to .dat file
        for (row=0; row<(lastMod*120); row++){ // lines
            for (col=0; col<560; col++) // words
                if(col<559)
                    fprintf(wfile, "%"SCNu16" ", *(img+row*560+col));
                else
                    fprintf(wfile, "%"SCNu16, *(img+row*560+col));
            if(row<(lastMod*120-1))
                fprintf(wfile, "\n");
        }//
        fclose(wfile);
    }
    printf("\n");
    fflush(stdout);

    if(xpci_getAbortProcess())
            return 1;
    if(xpci_modLoadConfigG(modMask, 0x7f, AMP_TP, 0) != 0){
        printf("%s() ERROR: writing global configuration (AMP_T)\n", __func__);
        free(daclImg[0]);
        return -1;
    }

    free(daclImg);
    return 0;
}






/////////////////////////////////


int imxpad_processIthlScanDataOTN(unsigned modMask, char *dirpath, unsigned ithl_min, unsigned ithl_max, unsigned *ithlval){

    //printf("%s() \n", __func__);

    FILE *rdfile;
    int pos = strlen(dirpath);
    char fname[pos+15];
    struct stat path_status;
    int modNb = xpci_getModNb(modMask);

    unsigned *ithlimg; // current ithl image
    unsigned *buffer; //buffer containing all ithl images
    
    int i, j, row, column;
    unsigned ithlVal,index;
    
    int LineNumber = 120*modNb;
    int ColumnNumber = 560;
    
    int scansize = ithl_max - ithl_min + 1;
    
    // check that directory exist
    if(stat(dirpath, &path_status)==0){
        if(!S_ISDIR(path_status.st_mode)){
            printf("%s() ERROR: %s is not a directory.\n", __func__, dirpath);
            return -1;
        }
    }
    else{
        printf("%s() ERROR: %s directory does not exist.\n", __func__, dirpath);
        return -1;
    }

    //allocate memory
    ithlimg = malloc(LineNumber*ColumnNumber*sizeof(unsigned)); // one image buffer
    buffer = malloc(LineNumber*ColumnNumber*scansize*sizeof(unsigned));//all images buffer

    // open ithl scan files and store them in a global buffer containing all images
    for(ithlVal=ithl_max; ithlVal>=ithl_min; ithlVal--){

        if(xpci_getAbortProcess()) return 1;

        index = ithlVal - ithl_min;

        // build name string
        sprintf(fname, "%s/ITHL_%d.dat", dirpath, ithlVal);
        // open file for reading
        if ((rdfile=fopen(fname, "r"))==NULL){
            printf("%s() ERROR: failed to open file %s.\n", __func__, fname);
            free(ithlimg);
            free(buffer);
            return -1;
        }
        // read file matrix
        printf(".");
        fflush(stdout);
        if(imxpad_readDataMatrix(rdfile, modMask, ithlimg)!=0){
            printf("%s ERROR: failed to read file %s.\n", __func__, fname);
            fclose(rdfile);
            free(ithlimg);
            free(buffer);
            return -1;
        }
        
        fclose(rdfile);

        //Scan each image and count the pixels with noise
        for (row=0;row< LineNumber;row++) {
            for (column=0;column<ColumnNumber;column++){
                //Writting a buffer containting the 64 read images
                buffer[index*LineNumber*ColumnNumber + (row*ColumnNumber+column)] = ithlimg[row*ColumnNumber+column];
                //if (row == 60 && column == 60)
                //    printf("%u ", ithlimg[row*ColumnNumber+column]);
            }
        }
    }

    //printf("\n");
    
    int diff_cur=0, diff_max=0;
    for (row=0;row<LineNumber;row++) {
        for (column=0;column<ColumnNumber;column++){

            if(xpci_getAbortProcess())
                return -1;

            unsigned ithlValue=ithl_min;
            diff_cur = 0;
            diff_max = 0;

            //Detect the noise peak (only count above 5 in order not to be affected by cosmics)
            for (ithlVal=ithl_max; ithlVal>=ithl_min; ithlVal--){

                if(xpci_getAbortProcess())
                    return -1;

                int index = ithlVal - ithl_min;

                if (index > 0 && index < scansize-1)
                    diff_cur = buffer[(index-1)*LineNumber*ColumnNumber + (row*(ColumnNumber)+column)] - buffer[(index+1)*LineNumber*ColumnNumber + (row*(ColumnNumber)+column)];
                else if (index == 0 || index == scansize-1)
                    diff_cur = 0;

                //if (row == 60 && column == 60)
                //printf("%d ", diff_cur);

                if (diff_max < diff_cur){
                    ithlValue = ithlVal;
                    diff_max = diff_cur;
                }
            }
            ithlimg[(row*(ColumnNumber)+column)] = ithlValue;
            //if (row == 60 && column == 60)
            //    printf("ithlValue = %u ", ithlValue);
        }
    }
    printf("\n");
    printf("\n");
    free(buffer);
    
    unsigned sum=0, count=0;
    float mean;
    //Mean Value of ITHL for each chip of every module
    for (i=0; i<modNb; i++)
        for (j=0; j<7; j++){ //chipNumber

            if(xpci_getAbortProcess())
                return 1;

            //cout << "chip= " << j << endl;
            for (row=i*120; row<(i+1)*120;row++){
                for (column=j*80; column<(j+1)*80; column++){
                    //cout << "chip= " << j << " row= " << row << " column= " << column << endl;
                    if (ithlimg[row*(ColumnNumber)+column]>=ithl_min){
                        sum += ithlimg[row*(ColumnNumber)+column];
                        count++;
                    }
                }
            }

            if (count > 0)
                mean = sum / count;
            else
                mean = 0;

            ithlval[i*7 + j] = (unsigned)mean;

            printf("%u ", ithlval[i*7 + j]);
            fflush(stdout);
            
            sum = 0;
            count = 0;
        }

    //printf("\n");
    //fflush(stdout);

    free (ithlimg);
    return 0;
}



