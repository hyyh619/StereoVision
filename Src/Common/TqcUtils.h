/**
@ TqcUtils.h
@ utility functions.
@ v1.0 2016.2.17 by Benet Huang
*/

#ifndef __TQC_UTILS_H
#define __TQC_UTILS_H

#include <vector>
#include "TqcOs.h"

#define TQC_MAX_PATH 256

class CLock
{
public:
    CLock();
    ~CLock();

public:
    bool    Lock();
    bool    UnLock();

protected:
    bool    Init();
    bool    UnInit();

private:
    LockerHandle    m_handle;
};

#ifdef WIN32
void AddFileList(const char *filePrefix, std::vector<char*> &fileList);
#endif

#endif /* __TQC_UTILS_H */