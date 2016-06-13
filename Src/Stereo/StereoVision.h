#ifndef __STEREO_VISION
#define __STEREO_VISION


#define ALGORITHM_OPTION    "--algorithm="
#define ALGORITHM_NAME_BM   "bm"
#define ALGORITHM_NAME_SGBM "sgbm"
#define ALGORITHM_NAME_HH   "hh"
#define ALGORITHM_NAME_VAR  "var"

#define MAX_DISPARITY_OPTION "--max-disparity="
#define BLOCK_SIZE_OPTION    "--blocksize="
#define SCALE_OPTION         "--scale="
#define NO_DISPLAY_OPTION    "--no-display"

typedef enum _enStereoAlgorithm
{
    STEREO_BM    = 0,
    STEREO_SGBM  = 1,
    STEREO_HH    = 2,
    STEREO_VAR   = 3,
    STEREO_VALID = -1
} enAlgorithm;

#endif /* __STEREO_VISION */