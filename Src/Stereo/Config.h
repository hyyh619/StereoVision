#ifndef __CONFIG_H
#define __CONFIG_H

#include <opencv2/opencv.hpp>

using namespace cv;


#ifndef SAVE_CAMERA_FRAME
#define SAVE_CAMERA_FRAME 0                              // Enable multithread for boosting performance. Disabled by default for android.
#endif

#endif /* __CONFIG_H */