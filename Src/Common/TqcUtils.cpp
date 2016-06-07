#include "TqcUtils.h"

/**
@ TqcUtils.cpp
@ utility functions.
@ v1.0 2016.2.17 by Benet Huang
*/

#include "TqcUtils.h"

CLock::CLock()
{
    Init();
}
CLock::~CLock()
{
    UnInit();
}

bool CLock::Lock()
{
    TqcOsAcquireMutex(m_handle);
    return true;
}
bool CLock::UnLock()
{
    TqcOsReleaseMutex(m_handle);
    return true;
}

bool CLock::Init()
{
    m_handle = TqcOsCreateMutex();
    if (!m_handle)
    {
        return false;
    }

    return true;
}

bool CLock::UnInit()
{
    TqcOsDeleteMutex(m_handle);
    return true;
}