//*****************************************************
//
// Author: Arkadiusz Dawiec
// Created: 04/04/2012
// *****************************************************

#include <stdio.h>
#include <unistd.h>

#include "xpci_interface.h"
#include "xpci_interface_expert.h"

#include "xpci_imxpad.h"

extern int xpci_systemType;
extern unsigned imxpad_postProc;
extern enum IMG_TYPE img_type;


// *******************************************************************************
// function to verify formatting of one image line
//
// *******************************************************************************
// 16 bits
int imxpad_checkImgLine_16bits(uint16_t *line){

    // check header
    if (*(line)!=0xaa55)
        return -1;
    // check trailer
    if (*(line+565)!=0xf0f0)
        return -1;
    // check line numbers
    if ((*(line+4)<1)||(*(line+4)>120))
        return -1;
    // check line length word
    if (*(line+2)!=0x0236)
        return -1;

    return 0;
}
// 32 bits
int imxpad_checkImgLine_32bits(uint16_t *line){

    // check header
    if (*(line)!=0xaa55)
        return -1;
    // check trailer
    if (*(line+1125)!=0xf0f0)
        return -1;
    // check line numbers
    if ((*(line+4)<1)||(*(line+4)>120))
        return -1;
    // check line length word
    if (*(line+2)!=0x0466)
        return -1;

    return 0;
}

// *******************************************************************************
// functions to convert raw images into organized matrices 
//
// *******************************************************************************
// 16 bits
int imxpad_raw2data_16bits(int modMask, uint16_t* oldImg, uint16_t* newImg){
    int ret = 0;

    switch(xpci_systemType){
    case IMXPAD_S70:
        ret = imxpad_extract2BImgData_S70(modMask, oldImg, newImg);
        break;
    case IMXPAD_S420:
        imxpad_extract2BImgData_S420(modMask, oldImg, newImg);
        break;
//    case IMXPAD_S340:
//    case IMXPAD_S540:
//       ret = imxpad_extract2BImgData_S540(modMask, oldImg, newImg);
//        break;
    case IMXPAD_S140:
        ret = imxpad_extract2BImgData_S140(modMask, oldImg, newImg);
        break;
    case IMXPAD_S540:
    case IMXPAD_S700:
    case IMXPAD_S1400:
        imxpad_extract2BImgData_S1400(modMask, oldImg, newImg);
        break;
    }

    return ret;
}


int imxpad_raw2data_16bits_v2(int modMask, uint16_t* oldImg, uint16_t** newImg,int imgNumber){
    int ret = 0;

    switch(xpci_systemType){
    case IMXPAD_S70:
        ret = imxpad_extract2BImgData_S70(modMask, oldImg, newImg[imgNumber]);
        break;
    case IMXPAD_S420:
        imxpad_extract2BImgData_S420(modMask, oldImg, newImg[imgNumber]);
        break;
    case IMXPAD_S340:    
        ret = imxpad_extract2BImgData_S540(modMask, oldImg, newImg[imgNumber]);
        break;
    case IMXPAD_S140:
        ret = imxpad_extract2BImgData_S140(modMask, oldImg, newImg[imgNumber]);
        break;
    case IMXPAD_S540:
    case IMXPAD_S700:
    case IMXPAD_S1400:
        imxpad_extract2BImgData_S1400_v2(modMask, oldImg, newImg);
        break;
    }

    return ret;
}




// 32 bits
int imxpad_raw2data_32bits(int modMask, uint16_t *oldImg, uint32_t *newImg){
    int ret = 0;

    switch(xpci_systemType){
    case IMXPAD_S70:
        ret = imxpad_extract4BImgData_S70(modMask, oldImg, newImg);
        break;
    case IMXPAD_S340:
        ret = imxpad_extract4BImgData_S540(modMask, oldImg, newImg);
        break;
    case IMXPAD_S140:
        ret = imxpad_extract4BImgData_S140(modMask, oldImg, newImg);
        break;
    case IMXPAD_S540:
    case IMXPAD_S700:
    case IMXPAD_S1400:
        imxpad_extract4BImgData_S1400(modMask, oldImg, newImg);
    }

    return ret;
}

// *******************************************************************************
// S140 detetctor
//
// the second module must be mirrored verticaly (row 119 <-> row 0) and horizontaly internaly in every chip (row 0 <-> row79)
// *******************************************************************************
// 16 bits
int imxpad_extract2BImgData_S140(int modMask, uint16_t *oldImg, uint16_t *newImg){
    int imgHeigth = 120*xpci_getModNb(modMask);
    int imgWidthNew = 560;
    int imgWidthOld = 566;
    int row, col, chip = 0;
    int newRow = 0;
    int headerOffset = 5;
    int module_id = 0;

    for(row=0; row<imgHeigth; row++){

        // check line format
        if(imxpad_checkImgLine_16bits((uint16_t *)(oldImg+row*imgWidthOld))!=0)
            return -1;

        module_id = oldImg[row*imgWidthOld+1];
        // assing new row id (mirror horizontaly the second module)
        if (module_id==1)
            newRow = oldImg[row*imgWidthOld+4]-1;
        else
            newRow = 240-oldImg[row*imgWidthOld+4];

        for(chip=0; chip<7; chip++){
            for(col=0; col<80; col++){
                if (module_id==1)
                    newImg[newRow*imgWidthNew+chip*80+col]=oldImg[row*imgWidthOld+chip*80+col+headerOffset];
                else
                    // mirror verticaly in the chips on the second module
                    newImg[newRow*imgWidthNew+chip*80+(79-col)]=oldImg[row*imgWidthOld+chip*80+col+headerOffset];
            }// for(col ...
        } // for(chip ...
    } // for(row ...

    return 0;
}

// 32 bits
int imxpad_extract4BImgData_S140(int modMask, uint16_t *oldImg, uint32_t *newImg){
    int imgHeigth = 120*xpci_getModNb(modMask);
    int imgWidthNew = 560;
    int imgWidthOld = 1126;
    int row, col, chip = 0;
    int newRow = 0;
    int headerOffset = 5;
    int module_id = 0;

    for(row=0; row<imgHeigth; row++){

        // check line format
        if(imxpad_checkImgLine_32bits((uint16_t *)(oldImg+row*imgWidthOld))!=0)
            return -1;

        module_id = oldImg[row*imgWidthOld+1];
        // assing new row id (mirror horizontaly the second module)
        /*if (module_id==1)
            newRow = (module_id-1)*120+oldImg[row*imgWidthOld+4]-1;
        else
            newRow = 360-(module_id-1)*120-oldImg[row*imgWidthOld+4]-1;*/
        if (module_id==1)
            newRow = oldImg[row*imgWidthOld+4]-1;
        else
            newRow = 240-oldImg[row*imgWidthOld+4];

        for(chip=0; chip<7; chip++){
            for(col=0; col<80; col++){
                if (module_id==1)
                    newImg[newRow*imgWidthNew+chip*80+col]=(oldImg[row*imgWidthOld+(chip*80+col)*2+1+headerOffset]<<16)+oldImg[row*imgWidthOld+(chip*80+col)*2+headerOffset];
                else
                    // mirror verticaly in the chips on the second module
                    newImg[newRow*imgWidthNew+chip*80+(79-col)]=(oldImg[row*imgWidthOld+(chip*80+col)*2+1+headerOffset]<<16)+oldImg[row*imgWidthOld+(chip*80+col)*2+headerOffset];

            } // for(col ...
        } // for(chip ...
    } // for(row ...

    return 0;
}

// *******************************************************************************
// S70 detetctor
//
// *******************************************************************************
// 16 bits
int imxpad_extract2BImgData_S70(int modMask, uint16_t *oldImg, uint16_t *newImg){
    int imgHeigth = 120*xpci_getModNb(modMask);
    int imgWidthNew = 560;
    int imgWidthOld = 566;
    int row, col, chip = 0;
    int headerOffset = 5;
    int i=0;

    // for(i=0; i<570; i++)
    //  printf("%d\n", oldImg[i]);


    for(row=0; row<imgHeigth; row++){

        // check line format
        if(imxpad_checkImgLine_16bits((uint16_t *)(oldImg+row*imgWidthOld))!=0)
            return -1;

        for(chip=0; chip<7; chip++){
            for(col=0; col<80; col++){
                newImg[row*imgWidthNew+chip*80+col]=oldImg[row*imgWidthOld+chip*80+col+headerOffset];
            } // for(col ...
        } // for(chip ...
    } // for(row ...

    return 0;
}

// *******************************************************************************
// S1400 detetctor
//
// *******************************************************************************

int imxpad_extract2BImgData_S1400(int modMask, uint16_t *oldImg, uint16_t *newImg){
    int imgHeigth = 120*xpci_getModNb(modMask);
    int imgWidthNew = 560;
    int imgWidthOld = 566;
    int row, col, chip = 0;
    int headerOffset = 5;
    int i=0;

    int newRow = 0;
    int rowOffset = 0;
    int module_id = 0;


    // for(i=0; i<570; i++)
    //  printf("%d\n", oldImg[i]);

    for(row=0; row<imgHeigth; row++){

        // check line format
        if(imxpad_checkImgLine_16bits((uint16_t *)(oldImg+row*imgWidthOld))!=0)
            return -1;

        module_id = oldImg[row*imgWidthOld+1];
        rowOffset = (module_id-1)*120;
        newRow = oldImg[row*imgWidthOld+4]-1 + rowOffset;


        for(chip=0; chip<7; chip++){
            for(col=0; col<80; col++){
                newImg[newRow*imgWidthNew+chip*80+col]=oldImg[row*imgWidthOld+chip*80+col+headerOffset];
            } // for(col ...
        } // for(chip ...
    } // for(row ...
    return 0;
}


int imxpad_extract2BImgData_S1400_v2(int modMask, uint16_t *oldImg, uint16_t **newImg){
    int imgHeigth = 120*xpci_getModNb(modMask);
    int imgWidthNew = 560;
    int imgWidthOld = 566;
    int row, col, chip = 0;
    int headerOffset = 5;
    int i=0;
    int imageNumber = 0;

    int newRow = 0;
    int rowOffset = 0;
    int module_id = 0;


    // for(i=0; i<570; i++)
    //  printf("%d\n", oldImg[i]);

    for(row=0; row<imgHeigth; row++){

        // check line format
        if(imxpad_checkImgLine_16bits((uint16_t *)(oldImg+row*imgWidthOld))!=0)
            return -1;

        module_id = oldImg[row*imgWidthOld+1];
        rowOffset = (module_id-1)*120;
        imageNumber = oldImg[row*imgWidthOld+3];
        newRow = oldImg[row*imgWidthOld+4]-1 + rowOffset;
        for(chip=0; chip<7; chip++){
            for(col=0; col<80; col++){
                newImg[imageNumber][newRow*imgWidthNew+chip*80+col]=oldImg[row*imgWidthOld+chip*80+col+headerOffset];
            } // for(col ...
        } // for(chip ...
    } // for(row ...
    return 0;
}


// 32 bits
int imxpad_extract4BImgData_S70(int modMask, uint16_t *oldImg, uint32_t *newImg){
    int imgHeigth = 120*xpci_getModNb(modMask);
    int imgWidthNew = 560;
    int imgWidthOld = 1126;
    int row, col, chip = 0;
    int headerOffset = 5;

    for(row=0; row<imgHeigth; row++){

        // check line format
        if(imxpad_checkImgLine_32bits((uint16_t *)(oldImg+row*imgWidthOld))!=0)
            return -1;

        for(chip=0; chip<7; chip++){
            for(col=0; col<80; col++){
                newImg[row*imgWidthNew+chip*80+col]=(oldImg[row*imgWidthOld+(chip*80+col)*2+1+headerOffset]<<16)+oldImg[row*imgWidthOld+(chip*80+col)*2+headerOffset];
            } // for(col ...
        } // for(chip ...
    } // for(row ...

    return 0;
}



int imxpad_extract4BImgData_S1400(int modMask, uint16_t *oldImg, uint32_t *newImg){
    int imgHeigth = 120*xpci_getModNb(modMask);
    int imgWidthNew = 560;
    int imgWidthOld = 1126;
    int row, col, chip = 0;
    int newRow = 0;
    int headerOffset = 5;
    int rowOffset = 0;
    int module_id = 0;
    int i=0;
    int newcol=0;
    int imageNumber;
    /*
  FILE *fd=NULL;
  
  fd=fopen("/home/imxpad/raw_B4_S1400.dat","w+");
  
  for(int i = 0; i<(120*20) ; i++ ){
    for(int j = 0; j<(560*2+6);j++)
        fprintf(fd,"0x%x ",oldImg[i*(560*2+6) + j]);
    fprintf(fd,"\n");
  }
  fclose (fd);
*/


    for(row=0; row<imgHeigth; row++){

        // check line format
        if(imxpad_checkImgLine_32bits((uint16_t *)(oldImg+row*imgWidthOld))!=0)
            return -1;

        module_id = oldImg[row*imgWidthOld+1];
        rowOffset = (module_id-1)*120;
        imageNumber = oldImg[row*imgWidthOld+3];
        newRow = oldImg[row*imgWidthOld+4]-1 + rowOffset;
        for(chip=0; chip<7; chip++){
            for(col=0; col<80; col++){
                newImg[newRow*imgWidthNew+chip*80+col]=(oldImg[row*imgWidthOld+(chip*80+col)*2+1+headerOffset] << 16) + oldImg[row*imgWidthOld+(chip*80+col)*2+headerOffset];
            } // for(col ...
        } // for(chip ...
    } // for(row ...

    return 0;
}




// *******************************************************************************
// S540 detector
//
// required swap between odd and even modules (i.e. mod 1 <-> mod2; mod3 <-> mod4)
// *******************************************************************************
// 16 bits
int imxpad_extract2BImgData_S540(int modMask, uint16_t *oldImg, uint16_t *newImg){
    int imgHeigth = 120*xpci_getModNb(modMask);
    int imgWidthNew = 560;
    int imgWidthOld = 566;
    int row, col, chip = 0;
    int newRow = 0;
    int headerOffset = 5;
    int rowOffset = 0;
    int module_id = 0;

    for(row=0; row<imgHeigth; row++){

        // check line format
        if(imxpad_checkImgLine_16bits((uint16_t *)(oldImg+row*imgWidthOld))!=0)
            return -1;

        module_id = oldImg[row*imgWidthOld+1];
        rowOffset = (module_id%2) ? (120) : -120;
        newRow = (module_id-1)*120+oldImg[row*imgWidthOld+4]-1 + rowOffset;
        for(chip=0; chip<7; chip++){
            for(col=0; col<80; col++){
                newImg[newRow*imgWidthNew+chip*80+col]=oldImg[row*imgWidthOld+chip*80+col+headerOffset];
            } // for(col ...
        } // for(chip ...
    } // for(row ...

    return 0;
}

// 32 bits
int imxpad_extract4BImgData_S540(int modMask, uint16_t *oldImg, uint32_t *newImg){
    int imgHeigth = 120*xpci_getModNb(modMask);
    int imgWidthNew = 560;
    int imgWidthOld = 1126;
    int row, col, chip = 0;
    int newRow = 0;
    int headerOffset = 5;
    int rowOffset = 0;
    int module_id = 0;

    for(row=0; row<imgHeigth; row++){

        // check line format
        if(imxpad_checkImgLine_32bits((uint16_t *)(oldImg+row*imgWidthOld))!=0)
            return -1;

        module_id = oldImg[row*imgWidthOld+1];
        rowOffset = (module_id%2) ? (120) : -120;
        newRow = (module_id-1)*120+oldImg[row*imgWidthOld+4]-1 + rowOffset;
        for(chip=0; chip<7; chip++){
            for(col=0; col<80; col++){
                newImg[newRow*imgWidthNew+chip*80+col]=(oldImg[row*imgWidthOld+(chip*80+col)*2+1+headerOffset]<<16)+oldImg[row*imgWidthOld+(chip*80+col)*2+headerOffset];
            } // for(col ...
        } // for(chip ...
    } // for(row ...

    return 0;
}

// *******************************************************************************

// *******************************************************************************
// S540 detector
//
// required swap between odd and even modules (i.e. mod 1 <-> mod2; mod3 <-> mod4)
// *******************************************************************************
// 16 bits
int imxpad_extract2BImgData_S420(int modMask, uint16_t *oldImg, uint16_t *newImg){
    int imgHeigth = 120*xpci_getModNb(modMask);
    int imgWidthNew = 560;
    int imgWidthOld = 566;
    int firstMod = xpci_getFirstMod(modMask);
    int row, col, chip = 0;
    int newRow = 0;
    int headerOffset = 5;
    int rowOffset = 0;
    int module_id = 0;

    for(row=0; row<imgHeigth; row++){

        // check line format
        if(imxpad_checkImgLine_16bits((uint16_t *)(oldImg+row*imgWidthOld))!=0)
            return -1;
        module_id = oldImg[row*imgWidthOld+1]-firstMod;
        rowOffset = (module_id%2) ? (120) : -120;
        newRow = (module_id-1)*120+oldImg[row*imgWidthOld+4]-1 + rowOffset;
        for(chip=0; chip<7; chip++){
            for(col=0; col<80; col++){
                newImg[newRow*imgWidthNew+chip*80+col]=oldImg[row*imgWidthOld+chip*80+col+headerOffset];
            } // for(col ...
        } // for(chip ...
    } // for(row ...

    return 0;
}

// 32 bits
int imxpad_extract4BImgData_S420(int modMask, uint16_t *oldImg, uint32_t *newImg){
    int imgHeigth = 120*xpci_getModNb(modMask);
    int imgWidthNew = 560;
    int imgWidthOld = 1126;
    int firstMod = xpci_getFirstMod(modMask);
    int row, col, chip = 0;
    int newRow = 0;
    int headerOffset = 5;
    int rowOffset = 0;
    int module_id = 0;

    for(row=0; row<imgHeigth; row++){

        // check line format
        if(imxpad_checkImgLine_32bits((uint16_t *)(oldImg+row*imgWidthOld))!=0)
            return -1;

        module_id = oldImg[row*imgWidthOld+1]-firstMod;
        rowOffset = (module_id%2) ? (120) : -120;
        newRow = (module_id-1)*120+oldImg[row*imgWidthOld+4]-1 + rowOffset;
        for(chip=0; chip<7; chip++){
            for(col=0; col<80; col++){
                newImg[newRow*imgWidthNew+chip*80+col]=(oldImg[row*imgWidthOld+(chip*80+col)*2+1+headerOffset]<<16)+oldImg[row*imgWidthOld+(chip*80+col)*2+headerOffset];
            } // for(col ...
        } // for(chip ...
    } // for(row ...

    return 0;
}




// function to increment ITHL values in a detector
int imxpad_incrITHL(unsigned modMask){

    unsigned modNb = xpci_getLastMod(modMask);
    int firstMod = xpci_getFirstMod(modMask);
    int lastMod    = xpci_getLastMod(modMask);
    unsigned curMod = 0;
    unsigned detRet[lastMod*7];
    unsigned curChip = 0;
    unsigned retMask = 0;
    unsigned localChipMask=0;
    int i = 0;

    usleep(5000);

    if(xpci_modReadConfigG(modMask, 0x7f, ITHL, &detRet[0]) != 0){
        printf(" errors while reading global configuration\n");
        return -1;
    }

    //printf("Current register values:\n");
    //for(curMod=0; curMod<modNb; curMod++){
    //  for(i=0; i<7; i++)
    //    printf("%02d ", detRet[curMod*7+i]);
    //  printf("\n");
    //}
    for(curMod=firstMod; curMod<=lastMod; curMod++){
        retMask = 0x01<<(curMod);
        localChipMask = 0x01;
        if(retMask & modMask)
        {
            for(curChip=0;curChip<7;curChip++){
                printf(".");
                fflush(stdout);
                if(xpci_modLoadConfigG(retMask, localChipMask, ITHL, detRet[curMod*7+curChip]+1) != 0)
                    printf(" errors while reading global configuration (module 0x%04x, chip 0x%02x)\n", retMask, localChipMask);
                localChipMask=localChipMask<<1;
            }
        }
        printf("\n");
    }

    if(xpci_modReadConfigG(modMask, 0x7f, ITHL, &detRet[0]) != 0){
        printf(" errors while reading global configuration\n");
        return -1;
    }

    //printf("\nNew register values:\n");
    //for(curMod=0; curMod<modNb; curMod++){
    //  for(i=0; i<7; i++)
    //    printf("%02d ", detRet[curMod*7+i]);
    //  printf("\n");
    //}

    return 0;
}

// *******************************************************************************
// function to decrement ITHL values in a detector
int imxpad_decrITHL(unsigned modMask){

    unsigned modNb = xpci_getLastMod(modMask);
    int firstMod = xpci_getFirstMod(modMask);
    int lastMod    = xpci_getLastMod(modMask);
    unsigned curMod = 0;
    unsigned detRet[(modNb+firstMod)*7];
    unsigned curChip = 0;
    unsigned retMask = 0;
    unsigned localChipMask=0;
    int i = 0;




    if(xpci_modReadConfigG(modMask, 0x7f, ITHL, &detRet[0]) != 0){
        printf(" errors while reading global configuration\n");
        return -1;
    }

    //printf("Current register values:\n");
    //for(curMod=0; curMod<modNb; curMod++){
    //  for(i=0; i<7; i++)
    //    printf("%02d ", detRet[curMod*7+i]);
    //  printf("\n");
    //}

    for(curMod=firstMod; curMod<=lastMod; curMod++){
        retMask = 0x01<<curMod;
        localChipMask = 0x01;
        if(	retMask & modMask)
        {
            for(curChip=0;curChip<7;curChip++){
                printf(".");
                fflush(stdout);
                if(xpci_modLoadConfigG(retMask, localChipMask, ITHL, detRet[curMod*7+curChip]-1) != 0)
                    printf(" errors while reading global configuration (module 0x%04x, chip 0x%02x)\n", retMask, localChipMask);
                localChipMask=localChipMask<<1;
            }
        }
        printf("\n");
    }

    if(xpci_modReadConfigG(modMask, 0x7f, ITHL, &detRet[0]) != 0){
        printf(" errors while reading global configuration\n");
        return -1;
    }

    //printf("\nNew register values:\n");
    //for(curMod=0; curMod<modNb; curMod++){
    //  for(i=0; i<7; i++)
    //    printf("%02d ", detRet[curMod*7+i]);
    //  printf("\n");
    //}

    return 0;
}

int imxpad_uploadExpWaitTimes(unsigned modMask, unsigned *pWaitTime, unsigned size){

    unsigned rem = size%50;
    unsigned floor = (size-rem)/50;
    int i,j =0;
    unsigned waitTimes[50];

    printf("size floor = %d, size reminder = %d\n", floor, rem);

    for(i=0;i<floor; i++){
        printf("sending values %d:%d\n",i*50, i*50+49);
        for(j=0;j<50;j++)
            waitTimes[j]=*(pWaitTime+i*50+j);
        xpci_modSendExpWaitTimes(modMask, i*50, i*50+49, waitTimes);
        usleep(1000000);
    }
    if(rem!=0){
        printf("sending values %d:%d\n",i*50, i*50+rem-1);
        for(j=0;j<rem;j++)
            waitTimes[j]=*(pWaitTime+i*50+j);
        for(j=rem;j<50;j++)
            waitTimes[j]=0;
        xpci_modSendExpWaitTimes(modMask, i*50, i*50+rem-1, waitTimes);
    }
}




// *******************************************************************************
// ALBA S1400 detector
// *******************************************************************************

int imxpad_writeFile2B_ALBA_S1400(unsigned modMask,char *fileName , uint16_t *newImg)
{
    FILE *fd=NULL;
    int col,row,mod;

    unsigned image[1200][1120];

    for(row=0;row<1200;row++)
    {
        for(col=0;col<1120;col++)
        {
            image[row][col] = 0;
        }
    }

    fd = fopen(fileName,"w+");
    if(fd == NULL)
    {
        printf("ERROR : Can not open file < %s >\n",fileName);
        return -1;
    }
    for(mod=0;mod<20;mod++)
    {
        if(((1<<mod) & modMask)==0) continue;
        for(row=0;row<120;row++)
        {
            for(col=0;col<560;col++)
            {
                image[((mod/2)*120) + row][(mod%2)*560 + col] = newImg[mod*560*120+col+row*560];
            }
        }
    }

    for(row=0;row<1200;row++)
    {
        for(col=0;col<1120;col++)
        {
            fprintf(fd,"%d ",image[row][col]);
        }
        fprintf(fd,"\n");
    }
    fflush(fd);
    fclose(fd);
    return 0;
}




int imxpad_writeFile4B_ALBA_S1400(unsigned modMask,char *fileName , uint32_t *newImg)
{
    FILE *fd=NULL;
    int col,row,mod;

    unsigned image[1200][1120];

    for(row=0;row<1200;row++)
        for(col=0;col<1120;col++)
            image[row][col]=0;

    fd = fopen(fileName,"w+");
    if(fd == NULL)
    {
        printf("ERROR : Can not open file < %s >\n",fileName);
        return -1;
    }
    for(mod=0;mod<20;mod++)
    {
        if(((1<<mod) & modMask)==0) continue;
        for(row=0;row<120;row++)
        {
            for(col=0;col<560;col++)
            {
                //image[(mod%2)*560 + col][((mod/2)*120) + row] = newImg[mod*560*120+col+row*120];
                image[((mod/2)*120) + row][(mod%2)*560 + col] = newImg[mod*560*120+col+row*560];
            }
        }
    }

    for(row=0;row<1200;row++)
    {
        for(col=0;col<1120;col++)
        {
            fprintf(fd,"%d ",image[row][col]);
        }
        fprintf(fd,"\n");
    }
    fflush(fd);
    fclose(fd);
    return 0;
}

int imxpad_raw_file_to_images(enum IMG_TYPE type, unsigned modMask, char *fpathOut, int startImg, int stopImg, int burstNumber){

    uint16_t *pRawBuffIn;
    uint16_t *pRawBuffOut;

    char fnameIn[100];
    char fnameOut[1000];

    int   lastMod = xpci_getLastMod(modMask);

    unsigned int imgSizeIn;
    unsigned int imgSizeOut;

    // determine size of the image
    if (type == B2){
        imgSizeIn = 120*566*lastMod*sizeof(uint16_t);
        imgSizeOut = 120*560*lastMod*sizeof(uint16_t);
    }
    else{
        imgSizeIn = 120*1126*lastMod*sizeof(uint16_t);
        imgSizeIn = 120*1120*lastMod*sizeof(uint16_t);
    }


    pRawBuffIn = malloc(imgSizeIn);
    pRawBuffOut = malloc(imgSizeOut);

    for (int i=startImg; i<stopImg; i++){


        sprintf(fnameIn,"/opt/imXPAD/tmp/burst_%d_img_%d.bin",burstNumber,i);
        FILE *imgIn = fopen(fnameIn,"rb");
        if(imgIn==-1){
            printf("ERROR => Can not open file < %s >\n",fnameIn);
            return -1;
        }

        //sprintf(fnameOut,"%s/img_%d.bin",fpath, i);
        sprintf(fnameOut,"%simg_%d.dat",fpathOut, i);
        FILE *imgOut = fopen(fnameOut,"wb");
        if(imgOut==-1){
            printf("ERROR => Can not open file < %s >\n",fnameOut);
            return -1;
        }

        fread(pRawBuffIn,imgSizeIn,1,imgIn);

        if (type == B2){
            imxpad_raw2data_16bits(modMask,pRawBuffIn,pRawBuffOut);
            printf("image number %d created\n",i);
            imxpad_writeFile2B_ALBA_S1400(modMask,fnameOut ,pRawBuffOut);
        }
        else{
            imxpad_raw2data_32bits(modMask,pRawBuffIn,pRawBuffOut);
        }

        fclose(imgIn);

    }

    free(pRawBuffIn);
    free(pRawBuffOut);

    return 0;
}


int imxpad_raw_file_to_buffer(enum IMG_TYPE type, unsigned modMask, void *pRawBuffOut , int numImageToAcquire, int burstNumber){

    uint16_t *pRawBuffIn = NULL;


    char fnameIn[100];

    int   lastMod = xpci_getLastMod(modMask);

    unsigned int imgSizeIn;

    // determine size of the image
    if (type == B2)
        imgSizeIn = 120*566*lastMod*sizeof(uint16_t);
    else
        imgSizeIn = 120*1126*lastMod*sizeof(uint16_t);

    pRawBuffIn = malloc(imgSizeIn);

    if (pRawBuffIn == NULL)
        return -1;

    sprintf(fnameIn,"/opt/imXPAD/tmp/burst_%d_img_%d.bin",burstNumber,numImageToAcquire);
    FILE *imgIn = fopen(fnameIn,"rb");
    if(imgIn==NULL){
        printf("ERROR => Can not open file < %s >\n",fnameIn);
        return -1;
    }
    else{
        fread(pRawBuffIn,imgSizeIn,1,imgIn);

        if (type == B2)
            imxpad_raw2data_16bits(modMask, pRawBuffIn, pRawBuffOut);
        else
            imxpad_raw2data_32bits(modMask, pRawBuffIn, pRawBuffOut);

        fclose(imgIn);
        free(pRawBuffIn);
        return 0;
    }

    return -1;
}
