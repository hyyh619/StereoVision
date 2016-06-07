/**
@ TqcUtils.h
@ utility functions.
@ v1.0 2016.2.17 by Benet Huang
*/

#ifndef __TQC_UTILS_H
#define __TQC_UTILS_H

#include "TqcOs.h"

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

#endif /* __TQC_UTILS_H */