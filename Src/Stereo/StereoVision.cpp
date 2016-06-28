// Stereo.cpp : Defines the entry point for the console application.
//

#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <Windows.h>
#include <iostream>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/core/ocl.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>

#include "TqcLog.h"
#include "Config.h"
#include "StereoVision.h"
#include "StereoCamera.h"
#include "StereoMatchAlgorithm.h"
#include "StereoUtils.h"

using namespace cv;

const char *g_windowName = "StereoVision";             // Name shown in the GUI window.

int  g_border           = 5;
int  g_cameraWidth      = 320;
int  g_cameraHeight     = 240;
int  g_windowWidth      = g_cameraWidth * 2 + g_border * 4;
int  g_windowHeight     = g_cameraHeight * 2 + g_border * 4;
Size g_imgSize          = Size(g_cameraWidth, g_cameraHeight);
Size g_camCalibrateSize = Size(g_cameraWidth, g_cameraHeight);


// Mouse event handler. Called automatically by OpenCV when the user clicks in the GUI window.
void OnMouse(int event, int x, int y, int, void*)
{
    // We only care about left-mouse clicks, not right-mouse clicks or mouse movement.
    if (event != CV_EVENT_LBUTTONDOWN)
        return;
}

int main(int argc, char *argv[])
{
    int          i = 0;
    VideoCapture leftCam;
    VideoCapture rightCam;
    Mat          leftFrame;
    Mat          rightFrame;
    Rect         dstRC;
    Mat          dstROI;

    if (!ParseCmd(argc, argv, g_option))
    {
        LOGE("%s(%d): cannot parse input arguments.", __FUNCTION__, __LINE__);
        return -1;
    }

    if (!StereoLoadCamParam(g_option.strIntrinsicFile,
                            g_option.strExtrinsicFile,
                            g_option.fScale,
                            g_imgSize,
                            g_camCalibrateSize,
                            &g_CamParam))
    {
        LOGE("%s(%d): cannot load camera's parameters(%s, %s).", __FUNCTION__, __LINE__,
             g_option.strIntrinsicFile, g_option.strExtrinsicFile);
        return -1;
    }

    if (!StereoOpenCam(leftCam, rightCam, g_cameraWidth, g_cameraHeight))
    {
        LOGE("%s(%d): cannot open stereo cameras.", __FUNCTION__, __LINE__);
        return -1;
    }

    // Create a GUI window for display on the screen.
    namedWindow(g_windowName); // Resizable window, might not work on Windows.
    resizeWindow(g_windowName, g_windowWidth, g_windowHeight);

    // Get OpenCV to automatically call my "onMouse()" function when the user clicks in the GUI window.
    setMouseCallback(g_windowName, OnMouse, 0);

    if (!StereoGetFrame(leftCam, rightCam, leftFrame, rightFrame))
    {
        return -1;
    }

    if (!StereoInitAlgorithm(leftFrame.channels(),
                             g_CamParam.roi1,
                             g_CamParam.roi2,
                             g_option.nNumDisparities,
                             g_option.nSADWindowSize,
                             g_imgSize.width,
                             g_option.algorithm))
    {
        return -1;
    }

    while (1)
    {
        Mat displayFrame = Mat(Size(g_windowWidth, g_windowHeight), CV_8UC3);
        displayFrame.empty();

        if (!StereoGetFrame(leftCam, rightCam, leftFrame, rightFrame))
        {
            return -1;
        }

        Mat img1r, img2r;
        remap(leftFrame, img1r, g_CamParam.map11, g_CamParam.map12, INTER_LINEAR);
        remap(leftFrame, img2r, g_CamParam.map21, g_CamParam.map22, INTER_LINEAR);

        leftFrame = img1r;
        leftFrame = img2r;

        Mat disp, disp8;

        int64 t = getTickCount();

        if (g_option.algorithm == TQC_STEREO_BM)
        {
            g_bm->compute(leftFrame, rightFrame, disp);
        }
        else if (g_option.algorithm == TQC_STEREO_SGBM || g_option.algorithm == TQC_STEREO_HH)
        {
            g_sgbm->compute(leftFrame, rightFrame, disp);
        }

        t = getTickCount() - t;
        LOGE("#%d---Time elapsed: %fms\n", ++i, t * 1000 / getTickFrequency());

        if (g_option.algorithm != TQC_STEREO_VAR)
            disp.convertTo(disp8, CV_8U, 255 / (g_option.nNumDisparities * 16.));
        else
            disp.convertTo(disp8, CV_8U);

        // Get the destination ROI (and make sure it is within the image!).
        dstRC  = Rect(g_border, g_border, g_cameraWidth, g_cameraHeight);
        dstROI = displayFrame(dstRC);
        leftFrame.copyTo(dstROI);

        // Get the destination ROI (and make sure it is within the image!).
        dstRC  = Rect(g_cameraWidth + g_border * 2, g_border, g_cameraWidth, g_cameraHeight);
        dstROI = displayFrame(dstRC);
        rightFrame.copyTo(dstROI);

        // Get the destination ROI (and make sure it is within the image!).
        dstRC  = Rect(g_border, g_border * 2 + g_cameraHeight, g_cameraWidth, g_cameraHeight);
        dstROI = displayFrame(dstRC);
        Mat dispColor;
        // disp8.convertTo(dispColor, displayFrame.type());
        cvtColor(disp8, dispColor, CV_GRAY2BGR);
        dispColor.copyTo(dstROI);

        imshow(g_windowName, displayFrame);

        // IMPORTANT: Wait for at least 20 milliseconds, so that the image can be displayed on the screen!
        // Also checks if a key was pressed in the GUI window. Note that it should be a "char" to support Linux.
        char keypress = waitKey(20);  // This is needed if you want to see anything!
        if (keypress == VK_ESCAPE)
        {
            // Escape Key
            // Quit the program!
            break;
        }
    }

    return 0;
}