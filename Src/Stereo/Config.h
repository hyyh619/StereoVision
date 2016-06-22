#ifndef __CONFIG_H
#define __CONFIG_H

#include <opencv2/opencv.hpp>

using namespace cv;


#ifndef SAVE_CAMERA_FRAME
#define SAVE_CAMERA_FRAME 0                              // Enable multi-thread for boosting performance. Disabled by default for android.
#endif

#ifdef WIN32
#define TQC_LOGICAL_CAM_LEFT_INDEX      1
#define TQC_LOGICAL_CAM_RIGHT_INDEX     0
#define TQC_INTERNAL_CAM_INDEX          2
#elif defined(ANDROID)
#define TQC_LOGICAL_CAM_LEFT_INDEX      0
#define TQC_LOGICAL_CAM_RIGHT_INDEX     1
#define TQC_INTERNAL_CAM_INDEX          2
#else
#define TQC_LOGICAL_CAM_LEFT_INDEX      0
#define TQC_LOGICAL_CAM_RIGHT_INDEX     1
#define TQC_INTERNAL_CAM_INDEX          2
#endif

#endif /* __CONFIG_H */