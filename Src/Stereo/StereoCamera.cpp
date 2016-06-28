#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <Windows.h>
#include <iostream>
#include <vector>
#include <string>

#include "TqcLog.h"
#include "Config.h"
#include "StereoCamera.h"

stCamParam g_CamParam;

bool StereoLoadCamParam(const char *strIntrinsicFile,
                        const char *strExtrinsicFile,
                        float fScale,
                        Size imgSize,
                        Size camCalibrateSize,
                        stCamParam *pOutCamParam)
{
    if (!strIntrinsicFile || !strExtrinsicFile)
        return false;

    // reading intrinsic parameters
    FileStorage fs(strIntrinsicFile, FileStorage::READ);
    if (!fs.isOpened())
    {
        LOGE("Failed to open file %s\n", strIntrinsicFile);
        return false;
    }

    Mat M1, D1, M2, D2;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;

    M1 *= fScale;
    M2 *= fScale;

    // If real image size is not equal to camera calibration size, we should scale it.
    if (imgSize != camCalibrateSize)
    {
        double sx = (double)imgSize.width / camCalibrateSize.width;
        double sy = (double)imgSize.height / camCalibrateSize.height;

        // adjust the camera matrix for the new resolution
        M1.at<double>(0, 0) *= sx;
        M1.at<double>(0, 2) *= sx;
        M1.at<double>(1, 1) *= sy;
        M1.at<double>(1, 2) *= sy;

        // adjust the camera matrix for the new resolution
        M2.at<double>(0, 0) *= sx;
        M2.at<double>(0, 2) *= sx;
        M2.at<double>(1, 1) *= sy;
        M2.at<double>(1, 2) *= sy;
    }

    fs.open(strExtrinsicFile, FileStorage::READ);
    if (!fs.isOpened())
    {
        LOGE("Failed to open file %s\n", strIntrinsicFile);
        return false;
    }

    Mat R, T;
    fs["R"] >> R;
    fs["T"] >> T;

    if (R.cols == 1 && R.rows == 3)
    {
        Rodrigues(R, R);
    }

    stereoRectify(M1, D1, M2, D2, imgSize, R, T,
                  pOutCamParam->R1, pOutCamParam->R2,
                  pOutCamParam->P1, pOutCamParam->P2,
                  pOutCamParam->Q, CALIB_ZERO_DISPARITY, -1,
                  imgSize, &pOutCamParam->roi1, &pOutCamParam->roi2);

    initUndistortRectifyMap(M1, D1, pOutCamParam->R1, pOutCamParam->P1, imgSize, CV_16SC2, pOutCamParam->map11, pOutCamParam->map12);
    initUndistortRectifyMap(M2, D2, pOutCamParam->R2, pOutCamParam->P2, imgSize, CV_16SC2, pOutCamParam->map21, pOutCamParam->map22);

    return true;
}

bool StereoOpenCam(VideoCapture &leftCam, VideoCapture &rightCam, int camWidth, int camHeight)
{
    // Try to set the camera resolution. Note that this only works for some cameras on
    // some computers and only for some drivers, so don't rely on it to work!

    leftCam.open(TQC_LOGICAL_CAM_LEFT_INDEX);
    leftCam.set(CV_CAP_PROP_FRAME_WIDTH, camWidth);
    leftCam.set(CV_CAP_PROP_FRAME_HEIGHT, camHeight);

    rightCam.open(TQC_LOGICAL_CAM_RIGHT_INDEX);
    rightCam.set(CV_CAP_PROP_FRAME_WIDTH, camWidth);
    rightCam.set(CV_CAP_PROP_FRAME_HEIGHT, camHeight);

    return true;
}

bool StereoGetFrame(VideoCapture &leftCam, VideoCapture &rightCam, Mat &leftFrame, Mat &rightFrame)
{
    leftCam >> leftFrame;
    rightCam >> rightFrame;

    if (leftFrame.empty() || rightFrame.empty())
    {
        LOGE("%s(%d): Couldn't grab the next camera frame.\n", __FUNCTION__, __LINE__);
        return false;
    }

    return true;
}