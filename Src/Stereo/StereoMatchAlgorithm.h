#ifndef __STEREO_MATCH_ALGORITHM_H
#define __STEREO_MATCH_ALGORITHM_H

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/types.hpp>

#include "StereoCamera.h"

using namespace cv;

#define TQC_MAX_DEPTH               10000.0f  // 10m


typedef enum _enStereoAlgorithm
{
    TQC_STEREO_BM    = 0,
    TQC_STEREO_SGBM  = 1,
    TQC_STEREO_HH    = 2,
    TQC_STEREO_VAR   = 3,
    TQC_STEREO_VALID = -1
} enAlgorithm;

typedef struct _stAlgorithmParam
{
    int         nNumDisparities;
    int         nSADWindowSize;
    int         nImgWidth;
    enAlgorithm selector;
}stAlgorithmParam;


// Function declaration
bool StereoInitAlgorithm(int nChannels,
                         Rect roi1,
                         Rect roi2,
                         int nNumDisparities,
                         int nSADWindowSize,
                         int imgWidth,
                         enAlgorithm selector = TQC_STEREO_SGBM);
bool StereoMatch(Mat left,
                 Mat right,
                 float fScale,
                 enAlgorithm selector,
                 stCamParam camParam,
                 Mat &disp);
Mat  StereoGetDisp8FromDisp(Mat disp, enAlgorithm selector, int nNumDisparities);
void StereoCalcDepthOfVirtualCopter(const Mat &disp, const Mat &Q, double d[3][3]);
void StereoFilterDisp(Mat &disp, Mat Q);


// Global variables' declaration
extern stAlgorithmParam g_algorithmParam;
extern Ptr<StereoBM>    g_bm;
extern Ptr<StereoSGBM>  g_sgbm;

#endif /* __STEREO_MATCH_ALGORITHM_H */