#ifndef __STEREO_CAMERA_H
#define __STEREO_CAMERA_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/core/ocl.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>

using namespace cv;


typedef struct _stCamParam
{
    Mat  R1, P1, R2, P2, Q;
    Rect roi1, roi2;
    Mat  map11, map12, map21, map22;
} stCamParam;


// Function Declaration
bool StereoLoadCamParam(const char *strIntrinsicFile,
                        const char *strExtrinsicFile,
                        float fScale,
                        Size imgSize,
                        Size camCalibrateSize,
                        stCamParam *pOutCamParam);
bool StereoOpenCam(VideoCapture &leftCam, VideoCapture &rightCam, int camWidth, int camHeight);
bool StereoGetFrame(VideoCapture &leftCam, VideoCapture &rightCam, Mat &leftFrame, Mat &rightFrame);


// Global variables' declaration
extern stCamParam   g_CamParam;
extern VideoCapture g_videoCapture1;
extern VideoCapture g_videoCapture2;

#endif /* __STEREO_CAMERA_H */