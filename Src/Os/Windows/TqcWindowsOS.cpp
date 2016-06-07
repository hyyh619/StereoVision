/**
@ TqcWindowsOS.cpp
@ OS API wrapper.
@ v1.0 2016.2.17 by Benet Huang
*/

#include <Windows.h>
#include "TqcOs.h"

void* TqcOsCreateThread(void *threadMain, void *pThread)
{
    HANDLE handle = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)threadMain, (void*)pThread, 0, NULL);
    return handle;
}

void TqcOsSleep(int millisecond)
{
    Sleep(millisecond);
}

LockerHandle TqcOsCreateMutex()
{
    CRITICAL_SECTION *sect = new CRITICAL_SECTION();
    ::InitializeCriticalSectionAndSpinCount(sect, 0);
    return (LockerHandle)sect;
}

void TqcOsDeleteMutex(LockerHandle handle)
{
    ::DeleteCriticalSection(LPCRITICAL_SECTION(handle));
    delete handle;
}

bool TqcOsAcquireMutex(LockerHandle handle)
{
    EnterCriticalSection(LPCRITICAL_SECTION(handle));
    return true;
}

void TqcOsReleaseMutex(LockerHandle handle)
{
    LeaveCriticalSection(LPCRITICAL_SECTION(handle));
}

unsigned int TqcOsGetMicroSeconds(void)
{
    LARGE_INTEGER   t1;
    LARGE_INTEGER   tc;
    unsigned int    time;

    QueryPerformanceFrequency(&tc);
    QueryPerformanceCounter(&t1);

    time = (unsigned int)(((double)t1.QuadPart / (double)tc.QuadPart) * 1000000);

    return time;
}