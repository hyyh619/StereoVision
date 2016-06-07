#include <pthread.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/syscall.h>

#include "TqcOs.h"

typedef void* (*pfnAndroidThreadDecl)(void*);

void* TqcOsCreateThread(void *threadMain, void *pThread)
{
    pthread_t handle;
    pthread_create(&handle, NULL, (pfnAndroidThreadDecl)threadMain, (void*)pThread);
    return (void*)handle;
}

void TqcOsSleep(int millisecond)
{
    usleep(1000 * millisecond);
}

LockerHandle TqcOsCreateMutex()
{
    pthread_mutex_t *mutex = new pthread_mutex_t;
    if (pthread_mutex_init(mutex, NULL) != 0)
        return 0;

    return (LockerHandle)mutex;
}

void TqcOsDeleteMutex(LockerHandle handle)
{
    pthread_mutex_destroy((pthread_mutex_t*)handle);
    delete (pthread_mutex_t*)handle;
}

bool TqcOsAcquireMutex(LockerHandle handle)
{
    pthread_mutex_lock((pthread_mutex_t*)handle);
    return true;
}

void TqcOsReleaseMutex(LockerHandle handle)
{
    pthread_mutex_unlock((pthread_mutex_t*)handle);
}

unsigned int TqcOsGetMicroSeconds(void)
{
    unsigned int time;
    struct timeval tv;

    /* Return the time of day in milliseconds. */
    gettimeofday(&tv, 0);
    time = (tv.tv_sec * 1000) + (tv.tv_usec / 1000);

    return time;
}
