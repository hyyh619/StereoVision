#ifndef __CONFIG_H
#define __CONFIG_H

#include <opencv2/opencv.hpp>

using namespace cv;


#ifndef SAVE_CAMERA_FRAME
#define SAVE_CAMERA_FRAME 0                              // Enable multi-thread for boosting performance. Disabled by default for android.
#endif

#define ALGORITHM_OPTION    "--algorithm="
#define ALGORITHM_NAME_BM   "bm"
#define ALGORITHM_NAME_SGBM "sgbm"
#define ALGORITHM_NAME_HH   "hh"
#define ALGORITHM_NAME_VAR  "var"

#define MAX_DISPARITY_OPTION "--max-disparity="
#define BLOCK_SIZE_OPTION    "--blocksize="
#define SCALE_OPTION         "--scale="
#define NO_DISPLAY_OPTION    "--no-display"

#endif /* __CONFIG_H */