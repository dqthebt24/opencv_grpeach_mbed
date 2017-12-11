#ifndef M_STORAGE_HPP
#define M_STORAGE_HPP

#include "mbed.h"
#include "rtos.h"
#include "FATFileSystem.h"
#include "SDBlockDevice_GR_PEACH.h"
#include "USBHostMSD.h"

enum mStorageError
{
    STORG_PASS, STORG_FAIL
};

class cStorage
{
private: 
    FATFileSystem *fs;
    SDBlockDevice_GR_PEACH sd;
    USBHostMSD usb;
    bool mounted;
public:
    /* Constructor */
    cStorage(char *name);
    
    /* Destructor */
    ~cStorage(void);
    
    /*Check sd card is connected*/
    mStorageError isConnectSdCard(); 
    mStorageError mountSdCard();
    mStorageError unmountSdCard(); 
};
#endif //M_STORAGE_HPP