/**
@ TqcTypes.h
@ OS API wrapper.
@ v1.0 2016.2.17 by Benet Huang
*/

#ifndef __OS_H
#define __OS_H

typedef void* LockerHandle;

void*           TqcOsCreateThread(void *threadMain, void *pThread);
void            TqcOsSleep(int millisecond);
LockerHandle    TqcOsCreateMutex();
void            TqcOsDeleteMutex(LockerHandle handle);
bool            TqcOsAcquireMutex(LockerHandle handle);
void            TqcOsReleaseMutex(LockerHandle handle);
unsigned int    TqcOsGetMicroSeconds(void);

#endif /* __OS_H */