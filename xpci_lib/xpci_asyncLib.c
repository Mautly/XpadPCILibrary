/**
 * \file                       xpci_asyncLib.c
 * \brief Management of asynchronous functions
 * \author Pierre-Yves Duval, Hector Perez-Ponce
 * \version 0.0
 * \date 14/12/2011
 * \updated 17/12/2013
 *
 *   This unit encapsulates the management of threads and structures used for
 * the asynchronous commands. It is compiled as a separate unit and the object
 * added to the xpc_lib.a library.
 *
 */
//==============================================================================
// PYD 14/2/2011
//============================================================================= 

#include "xpci_interface.h"
#include "xpci_interface_expert.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>


/**\brief  
 * Structure to contain the parameters related to image size and address
 */
typedef struct {
    enum IMG_TYPE type;
    int  moduleMask;
    int  nbChips;
    void *data;  // used for one image reading
    void **pBuff;// used for a sequence of n images reading
    int  nloop;
    int  firstTimeout;
    int  nbImg;
} READ_IMG_PARA;

/**\brief 
 * Structure to contain the data relative to the detector exposition
 */
typedef struct {
    int expose;    // flag to use expose+read(1) or just read(0)
    int gateMode;
    int gateLength;
    int timeUnit;
    // timeout is the MAX DELAY for
    //    one single read image in single reading
    //    the duration of all images reading in sequence of images acquisition
    int timeout;
} EXPOSE_PARA;

/**\brief 
 * General structure keeping all the needed parameters for an sync command
 */
typedef struct {
    int           timeout;     // not yet used now
    READ_IMG_PARA *readPara;   // parameter used for the read function
    EXPOSE_PARA   *exposePara;
    int           *userPara;
    int           (*cbFunc)(int myint, void *dum);
} READ_CB_STRUCT;

//*********************************************************************************
//                GLOBALS ASYNC READING CallBack DATA
//*********************************************************************************
//* only one async call accepted at a time
static pthread_t      readThread;
static int            readPending;

static READ_CB_STRUCT cbPara;
static READ_IMG_PARA  readPara;
static EXPOSE_PARA    exposePara;

/*******************************************
 * Default read callback function
 *******************************************/
static int defaultCB(int dumRet, void *st){
    printf("WARNING: default read callBack read return status was %d\n",dumRet);
    return dumRet;
}

/**\fn int xpci_asyncReadStatus()
 * Function to test if the detector is in use (read is pending)
 * \return [0]not pending      [1]pending
 *///===========================================================
int xpci_asyncReadStatus(){
    return readPending;
}

// Function executed in a separate thread that calls xpci_readOneImage()
// in async mode
//======================================================================
static void readImagesThread(void *st){
    int ret, res;
    READ_CB_STRUCT *cbPara =   (READ_CB_STRUCT *) st;
    readPending = 1;

    // we are not using this parameter to day but could be usefull in future
    printf("Read thread starting with timeout of %d\n", cbPara->timeout);

    if (cbPara->readPara->nloop==0){ // single image read
        // executes the read or get function depending if the expose function has to be used
        if (cbPara->exposePara->expose==0)
            ret = xpci_readOneImage(cbPara->readPara->type,
                                    cbPara->readPara->moduleMask,
                                    cbPara->readPara->nbChips,
                                    cbPara->readPara->data);
        else
            ret = xpci_getOneImage(cbPara->readPara->type,
                                   cbPara->readPara->moduleMask,
                                   cbPara->readPara->nbChips,
                                   cbPara->readPara->data,
                                   cbPara->exposePara->gateMode,
                                   cbPara->exposePara->gateLength,
                                   cbPara->exposePara->timeUnit,
                                   cbPara->exposePara->timeout);
        // call the CB function
    }
    else { // sequence of images to read
        /*ret =  xpci_getImgSeq(cbPara->readPara->type,
              cbPara->readPara->moduleMask,
              cbPara->readPara->nbChips,
              cbPara->exposePara->gateMode,
              cbPara->exposePara->gateLength,
              cbPara->exposePara->timeUnit,
              cbPara->readPara->nloop,
              cbPara->readPara->pBuff,
              cbPara->readPara->firstTimeout);*/
		ret = xpci_getImgSeq_SSD_imxpad(cbPara->readPara->type,
									cbPara->readPara->moduleMask,
									cbPara->readPara->nbImg,
									cbPara->exposePara->gateMode);
		/*
		
        ret =  xpci_getImgSeq(cbPara->readPara->type,
                              cbPara->readPara->moduleMask,
                              cbPara->readPara->nbChips,
                              cbPara->readPara->nbImg,
                              cbPara->readPara->pBuff,
                              cbPara->exposePara->gateMode,
                              cbPara->exposePara->gateLength,
                              cbPara->exposePara->timeUnit,
                              cbPara->readPara->firstTimeout);
		*/
    }
    res = cbPara->cbFunc(ret, cbPara->userPara);
    readPending = 0;
    pthread_exit(NULL);
}

// Function to read the detector in assync mode with a CB function and a timeout
// in seconds
// if expose !=0 the exposition is included in the process 
// if expose ==0 just the read image is done
// parameters : sequence 0=single read    1=multiple read
//====================================================================================
static int processImagesAs(int sequence,
                           enum IMG_TYPE type, int moduleMask, int nbChips,
                           void *data,void **pBuff,
                           int (*cbFunc)(int myint, void *dum), int timeout,
                           int expose, int gateMode, int gateLength, int timeUnit,
                           int nloop, int firstTimeout,
                           void *userPara, int nImg){
    int ret;

    if (readPending !=0 ){
        printf("ERROR: Operation denied, Can't start an access to PCIe while an async reading is pending in %s\n", __func__);
        return -1;
    }

    // should release resources of last call
    if (readThread!=0){
        // no need to kill if we are here because readPending was tested to =0
        pthread_join(readThread, NULL);
        readThread=0;
    }

    memset(&exposePara,0,sizeof(EXPOSE_PARA));
    memset(&readPara,  0,sizeof(READ_IMG_PARA));
    memset(&cbPara,    0,sizeof(READ_CB_STRUCT));
    //set CB parameters and async exec in thread parameters
    if (cbFunc==NULL)
        cbPara.cbFunc = defaultCB;
    else
        cbPara.cbFunc   = cbFunc;
    cbPara.timeout    = timeout;

    // set exposition parameters values
    exposePara.expose     = expose;
    exposePara.gateMode   = gateMode;
    exposePara.gateLength = gateLength;
    exposePara.timeUnit   = timeUnit;
    cbPara.exposePara = &exposePara;

    // set reading parameters values
    readPara.type        = type;
    readPara.moduleMask  = moduleMask;
    readPara.nbChips     = nbChips;
    readPara.nbImg       = nImg;
    
    if (sequence){
        readPara.pBuff        = pBuff;
        readPara.nloop        = nloop;
        readPara.firstTimeout = firstTimeout;
    }
    else {
        readPara.data      = data;
        readPara.nloop     = 0;
    }
    cbPara.readPara      = &readPara;

    // user defined parameters
    cbPara.userPara = userPara;

    if (ret = pthread_create(&readThread,
                             NULL,
                             readImagesThread,
                             (void*)&cbPara) !=0){
        printf("ERROR: Thread creation failed in %s\n", __func__);
        return -1;
    }
    else {
        printf("OK: Async thread creation is success\n");
        while(readPending==0) // wait to be sure the thread is started
            pthread_yield(); // let opportunity for thread real start
        //printf("OK reading thread is active\n");
        while(get_flagStartExpose() == 0);
        if(get_flagStartExpose() == 1)
				return 0;
	    else
				return -1;
    }
    // suspend to be sure the thread is started before coming back to main()
    usleep(50);
}

/**
 * \fn int   xpci_readOneImageAs(enum IMG_TYPE type, int moduleMask, int nbChips, void * data,int (*cbFunc)(int myint, void *dum), int timeout, void *userPara)
 * \brief Reads one image in async mode
 * \param enum IMG_TYPE type       Type of image to read 2B or 4B
 * \param int moduleMask           Modules to read
 * \param int nbChips              Number of chips per module
 * \param void *data               Pointer to the buffer where data should be received
 * \param int (*cbFunc)(int myint, void *dum) Callback function
 *
 * Note: the function receives the read images returned status in myInt and userPara in dum
 * \param int timeout Not used yet
 * \param void *userPara Pointer to a structure that will be passed to the callback function
 * \return [0]Reading started with success [-1]Reading lauch failed
*///==============================================================================
int   xpci_readOneImageAs(enum IMG_TYPE type, int moduleMask, int nbChips, 
                          void *data,int (*cbFunc)(int myint, void *dum), int timeout,
                          void *userPara){
    // without exposition expose flag = 0
    return processImagesAs(0,
                           type, moduleMask,nbChips,
                           data, 0,
                           cbFunc, timeout,
                           0, 0, 0, 0,  // exposition specific
                           0, 0, // sequence specific
                           userPara, 1); //number of images = 1
}
/**
 * \fn int   xpci_getOneImageAs(enum IMG_TYPE type, int moduleMask, int nbChips, void *data,int (*cbFunc)(int myint, void *dum), int timeout,int gateMode, int gateLength, int timeUnit,void * userPara)
 * \brief Reads one image in async mode with automatic exposition
 * \param enum IMG_TYPE type       Type of image to read 2B or 4B
 * \param int moduleMask           Modules to read
 * \param int nbChips              Number of chips per module
 * \param void *data               Pointer to the buffer where data should be received
 * \param int (*cbFunc)(int myint, void *dum) Callback function
 * Note: the function receives the read images returned status in myInt and userPara in dum
 * \param int gateMode
 * \param int gateLength
 * \param int timeUnit
 * \param int timeout Not used yet
 * \param void *userPara Pointer to a structure that will be passed to the callback function
 * \return [0]Reading started with success [-1]Reading lauch failed
*///==============================================================================
int   xpci_getOneImageAs(enum IMG_TYPE type, int moduleMask, int nbChips, void *data,
                         int (*cbFunc)(int myint, void *dum), int timeout,
                         int gateMode, int gateLength, int timeUnit,
                         void * userPara){
    return processImagesAs(0,
                           type, moduleMask,nbChips,
                           data, 0,
                           cbFunc, timeout,
                           1, gateMode, gateLength, timeUnit, //  expose flag =1
                           0, 0, // sequence specific
                           userPara,1);   //number of images = 1
}

/**
 * \fn int   xpci_getImgSeqAs(enum IMG_TYPE type, int moduleMask,int nImg)
 * \brief Start asynchronous exposition
 * \param enum IMG_TYPE type        Type of image to read 2B or 4B
 * \param int modMask               Modules to read
 * \param int nImg                  Total number of images requested in exposure parameters
 * \return [0]Reading finished with success [-1] Reading failed
*///==============================================================================
int   xpci_getImgSeqAsync(enum IMG_TYPE type, int moduleMask, int nImg, int burstNumber){

    printf("INSIDE getImgSeqAs\n");

    return processImagesAs(1,  //sequence flag = 1
                           type, moduleMask,7,
                           0,NULL,
                           NULL, 0,
                           0, burstNumber, 0, 0, //  expose flag not used
                           1, 8000, // sequence specific data
                           NULL, nImg);
}





void xpci_PreProcessGeometricalCorrections(){
    system ("sh /opt/imXPAD/geom_correction/Interpolator_init.sh");
}




/**
 * \fn int xpci_getAsyncImage(enum IMG_TYPE type, int modMask, int nChips,int nImg, void *pImg, int imageToGet)
 * \brief Reads one image in async mode with automatic exposition
 * \param enum IMG_TYPE type        Type of image to read 2B or 4B
 * \param int modMask               Modules to read
 * \param int nChips                Number of chips per module
 * \param int nImg                  Total number of images requested in exposure parameters
 * \param void * pImg               Pointer to the buffer where data should be received
 * \param int imageToGet            Number of the image to be read
 * \param void * pImgCorr			Pointer to the buffer where data geometrical corrected should be received
 * \param int geomCorr				Flag to enable or disable geometrical corrections
 * \return [0]Reading finished with success [-1] Reading failed
*///==============================================================================

int xpci_getAsyncImageFromSharedMemory(enum IMG_TYPE type, int modMask, int nChips,int nImg, void *pImg, int imageToGet, void *pImgCorr, int geomCorr){

    int             modNb = xpci_getModNb(modMask);

    // Variables for Async Reading
    int             fd;
    unsigned int    *imageNumber;
    int             numPixels;
    unsigned short  *image16;
    unsigned int    *image32;


    numPixels = 120*560*modNb;

    printf("type = %d ",type);
    printf("type = %d ",modMask);
    printf("type = %d ",nChips);
    printf("type = %d\n",imageToGet);

    //**************** Start of async variable set ****************
    //**************** imageNumber ****************                 //Last image acquired
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
                                         PROT_READ,
                                         MAP_SHARED, fd, 0 );
    if( imageNumber == MAP_FAILED ) {
        fprintf( stderr, "imageNumber mmap failed [imageNumber %s()]:%s\n",__func__,
                 strerror( errno ) );
        return -1;
    }

    close(fd);

    //**************** images ****************                     //Shared memory where images will be stored
    // Create a new memory object
    fd = shm_open( "/images", O_RDWR | O_CREAT, 0777 );
    if( fd == -1 ) {
        fprintf( stderr, "Open failed [images %s()]:%s\n",__func__,
                 strerror( errno ) );
        return -1;
    }

    // Map the memory object
    if(type==B2){
        // Set the memory object's size
        if( ftruncate( fd, sizeof( *image16 )*nImg*numPixels ) == -1 ) {
            fprintf( stderr, "ftruncate: [images %s()]:%s\n",__func__,
                     strerror( errno ) );
            return -1;
        }
        image16 = (unsigned short *) mmap( NULL, sizeof( *image16 )*nImg*numPixels,
                                           PROT_READ,
                                           MAP_SHARED, fd, 0 );
        if( image16 == MAP_FAILED ) {
            fprintf( stderr, "image mmap failed: [images %s()]:%s\n",__func__,
                     strerror( errno ) );
            return -1;
        }
    }
    else{
        // Set the memory object's size
        if( ftruncate( fd, sizeof( *image32 )*nImg*numPixels ) == -1 ) {
            fprintf( stderr, "ftruncate: %s\n",
                     strerror( errno ) );
            return -1;
        }
        image32 = (unsigned int *) mmap( NULL, sizeof( *image32 )*nImg*numPixels,
                                         PROT_READ,
                                         MAP_SHARED, fd, 0 );
        if( image32 == MAP_FAILED ) {
            fprintf( stderr, "image mmap failed: %s\n",
                     strerror( errno ) );
            return -1;
        }
    }

    close(fd);

    //**************** End of async variable set ****************

    //Returning the requested image after reading the shared memory.
    if (imageToGet >= 0){

        if (imageToGet < imageNumber[0]){
            if(type==B2){
                for(int i=0; i<numPixels; i++)
                    ((uint16_t *)pImg)[i] = image16[imageToGet*numPixels + i];
                if(geomCorr){
                    unsigned int buf[numPixels];
                    FILE *fileBin=fopen("/opt/imXPAD/geom_correction/matrix.raw","wb");
                    if(fileBin == NULL) {
                        printf("\nFile for geometrical correction could not be created\n");
                        return -1;
                    }
                    for(int i=0; i<numPixels; i++){
                        buf[i] = (unsigned int) image16[imageToGet*numPixels + i];
                        fwrite(&buf[i], sizeof(unsigned int), 1, fileBin);
                    }
                    fclose(fileBin);
                }
                munmap(image16,sizeof( *image16 )*nImg*numPixels);
            }
            else{
                for(int i=0; i<numPixels; i++)
                    ((uint32_t *)pImg)[i] = image32[imageToGet*numPixels + i];
                if(geomCorr){
                    FILE *fileBin=fopen("/opt/imXPAD/geom_correction/matrix.raw","wb");
                    if(fileBin == NULL) {
                        printf("\nFile for geometrical correction could not be created\n");
                        return -1;
                    }
                    for(int i=0; i<numPixels; i++)
                        fwrite(&image32[imageToGet*numPixels + i], sizeof(unsigned int), 1, fileBin);
                    fclose(fileBin);
                }
                munmap(image32,sizeof( *image32 )*nImg*numPixels);

            }
            if (geomCorr){
                system ("sh /opt/imXPAD/geom_correction/Interpolator.sh");
                FILE *fileBin=fopen("/opt/imXPAD/geom_correction/image_corrected.raw","rb");
                if(fileBin == NULL) {
                    printf("\nFile for geometrical correction could not be readed\n");
                    return -1;
                }
                float var;
                for(int i=0; i<(582*1157); i++){
                    fread(&var, sizeof(float), 1, fileBin);
                    ((float *)pImgCorr)[i]=var;
                }
                //printf ("\nInside library \n");
                fclose(fileBin);
            }
            munmap(imageNumber,sizeof( *imageNumber ));
            return 0;
        }
        else{
            printf("\nCurrent acquired image = %u\n",imageNumber[0]);
            munmap(imageNumber,sizeof( *imageNumber ));
            return -1;
        }
    }
    else{
        printf("\nNegative numbers doesn't make sense\n");
        return -1;
    }
}

/**
 * \fn int xpci_getAsyncImage(enum IMG_TYPE type, int modMask, int nChips,int nImg, void *pImg, int imageToGet)
 * \brief Reads one image in async mode with automatic exposition
 * \param enum IMG_TYPE type        Type of image to read 2B or 4B
 * \param int modMask               Modules to read
 * \param void * pImg               Pointer to the buffer where data should be received
 * \param int imageToGet            Number of the image to be read
 * \return [0]Reading finished with success [-1] Reading failed
*///==============================================================================

int xpci_getAsyncImageFromDisk(enum IMG_TYPE type, int modMask, void *pImg, int imageToGet, int numBurst){
    imxpad_raw_file_to_buffer(type, modMask, pImg, imageToGet, numBurst);
}

/**
 * \fn int xpci_getNumberLastAcquiredAsyncImage()
 * \brief Returns the number of the last acquired asynchronous image
 * \return number of the last acquired asynchronous image
*///==============================================================================
int xpci_getNumberLastAcquiredAsyncImage(){

    // Variables for Async Reading
    int             fd;
    unsigned int    *imageNumber;
    int             ret;

    //**************** Start of async variable set ****************
    //**************** imageNumber ****************                 //Last image acquired
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
    ret = imageNumber[0];
    munmap(imageNumber,sizeof( *imageNumber ));

    return ret;

}

void xpci_clearNumberLastAcquiredAsyncImage(){
	// Variables for Async Reading
    int             fd;
    unsigned int    *imageNumber;
    int             ret;

    //**************** Start of async variable set ****************
    //**************** imageNumber ****************                 //Last image acquired
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
    imageNumber[0] = -1;
    //printf("Shared image number resetted.\n");
    munmap(imageNumber,sizeof( *imageNumber ));
}

void xpci_cleanSharedMemory(){

    shm_unlink( "/imageNumber" );
    printf("Share memory unlinked\n");
    shm_unlink( "/images" );
    printf("Share memory unlinked\n");
    return 0;
}

void xpci_cleanSSDImages(unsigned int burstNumber, unsigned int imagesNumber){
	char str[200];
	sprintf(str,"rm /opt/imXPAD/tmp/burst_%d_img_*.bin",burstNumber);
    system(str);
}