#ifndef __TQC_LOG_H
#define __TQC_LOG_H

#include <string>
#include <stdio.h>
#include <stdarg.h>

#define LOG_ZONE            0xFFFF

#define FACE_ERROR          0x0001
#define FACE_WARNING        0x0002
#define FACE_INFO           0x0003
#define FACE_DEBUG          0x0004

#define LOG_LEVEL           0x0001

#ifdef TRACER_NO_LOG
#define TRACER_LOG(level, fmt, ...) {}
#else
#define TRACER_LOG(level, fmt, ...) FACE_LOG(level, FACE_TRACER, fmt, ##__VA_ARGS__)
#endif

#define LOGE(fmt, ...) FACE_LOG(FACE_ERROR, LOG_ZONE, fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) FACE_LOG(FACE_WARNING, LOG_ZONE, fmt, ##__VA_ARGS__)
#define LOGI(fmt, ...) FACE_LOG(FACE_INFO, LOG_ZONE, fmt, ##__VA_ARGS__)
#define LOGD(fmt, ...) FACE_LOG(FACE_DEBUG, LOG_ZONE, fmt, ##__VA_ARGS__)

#if defined(WIN32) || defined(__APPLE__)
#define FACE_LOG(level, zone, fmt, ...)\
        do\
        {\
            if (zone & LOG_ZONE)\
            {\
                if (level == FACE_ERROR && level <= LOG_LEVEL)\
                {\
                    printf(#zone);\
                    printf(":ERROR: ");\
                    printf(fmt, ##__VA_ARGS__);\
                    printf("\n");\
                }\
                if (level == FACE_WARNING && level <= LOG_LEVEL)\
                {\
                    printf(#zone);\
                    printf(":WARN: ");\
                    printf(fmt, ##__VA_ARGS__);\
                    printf("\n");\
                }\
                if (level == FACE_INFO && level <= LOG_LEVEL)\
                {\
                    printf(#zone);\
                    printf(":INFO: ");\
                    printf(fmt, ##__VA_ARGS__);\
                    printf("\n");\
                }\
                if (level == FACE_DEBUG && level <= LOG_LEVEL)\
                {\
                    printf(#zone);\
                    printf(":DEBUG: ");\
                    printf(fmt, ##__VA_ARGS__);\
                    printf("\n");\
                }\
            }\
        }while(0)

#elif defined(ANDROID)

#include <android/log.h>

#define FACE_LOG(level, zone, fmt, ...) \
        do\
        {\
            if (zone & LOG_ZONE)\
            {\
                if (level == FACE_ERROR && level <= LOG_LEVEL)\
                {\
                    __android_log_print(ANDROID_LOG_ERROR, "TqcVision", fmt, ## __VA_ARGS__);\
                }\
                if (level == FACE_WARNING && level <= LOG_LEVEL)\
                {\
                    __android_log_print(ANDROID_LOG_WARN, "TqcVision", fmt, ## __VA_ARGS__);\
                }\
                if (level == FACE_INFO && level <= LOG_LEVEL)\
                {\
                    __android_log_print(ANDROID_LOG_INFO, "TqcVision", fmt, ## __VA_ARGS__);\
                }\
                if (level == FACE_DEBUG && level <= LOG_LEVEL)\
                {\
                    __android_log_print(ANDROID_LOG_DEBUG, "TqcVision", fmt, ## __VA_ARGS__);\
                }\
            }\
        }while(0)

#else
#define FACE_LOG(level, zone, fmt, ...) {}
#endif

#endif // __TQC_LOG_H