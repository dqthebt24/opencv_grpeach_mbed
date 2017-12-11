#include "mStorage.hpp"

cStorage::cStorage(char *name)
{
    fs = new FATFileSystem(name);
    mounted = false;
}

cStorage::~cStorage(void)
{
    if( mounted == true)
    {
        fs->unmount();
        delete fs;
    }
}

mStorageError cStorage::isConnectSdCard()
{
    if (sd.connect()) 
    {
        return STORG_PASS;
    }
    return STORG_FAIL;
}

mStorageError cStorage::mountSdCard()
{
    if(fs == NULL)
    {
        return STORG_FAIL;
    }
    else if( fs->mount(&sd) == 0 )
    {
        mounted = true;
        return STORG_PASS;
    }
    return STORG_FAIL;
}

mStorageError cStorage::unmountSdCard()
{
    if( mounted == true )
    {
        fs->mount(&sd);
        return STORG_PASS;
    }
    return STORG_FAIL;
}