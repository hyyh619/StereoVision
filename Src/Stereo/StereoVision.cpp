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

using namespace cv;

#define ALGORITHM_OPTION    "--algorithm="
#define ALGORITHM_NAME_BM   "bm"
#define ALGORITHM_NAME_SGBM "sgbm"
#define ALGORITHM_NAME_HH   "hh"
#define ALGORITHM_NAME_VAR  "var"

#define MAX_DISPARITY_OPTION "--max-disparity="
#define BLOCK_SIZE_OPTION    "--blocksize="
#define SCALE_OPTION         "--scale="

const char *g_windowName        = "StereoVision";      // Name shown in the GUI window.
const char *g_intrinsicFileName = 0;
const char *g_extrinsicFileName = 0;
int        g_border             = 5;
int        g_windowWidth        = 1280 + g_border * 4;
int        g_windowHeight       = 720 + g_border * 4;
int        g_cameraWidth        = 640;
int        g_cameraHeight       = 360;
Size       g_imgSize            = Size(g_cameraWidth, g_cameraHeight);

enAlgorithm     g_algorithm      = STEREO_SGBM;
int             g_SADWindowSize  = 0;
int             g_numDisparities = 0;
float           g_scale          = 1.f;
char            *g_algorithmName = NULL;
Ptr<StereoBM>   g_bm             = StereoBM::create(16, 9);
Ptr<StereoSGBM> g_sgbm           = StereoSGBM::create(0, 16, 3);
Mat             g_R1, g_P1, g_R2, g_P2, g_Q;
Rect            g_roi1, g_roi2;
Mat             g_map11, g_map12, g_map21, g_map22;

// Mouse event handler. Called automatically by OpenCV when the user clicks in the GUI window.
void OnMouse(int event, int x, int y, int, void*)
{
    // We only care about left-mouse clicks, not right-mouse clicks or mouse movement.
    if (event != CV_EVENT_LBUTTONDOWN)
        return;
}

bool ParseCmd(int argc, char *argv[])
{
    if (argc < 3)
    {
        return false;
    }

    for (int i = 1; i < argc; i++)
    {
        if (strncmp(argv[i], ALGORITHM_OPTION, strlen(ALGORITHM_OPTION)) == 0)
        {
            g_algorithmName = argv[i] + strlen(ALGORITHM_OPTION);
            g_algorithm     = strcmp(g_algorithmName, ALGORITHM_NAME_BM) == 0 ? STEREO_BM :
                              strcmp(g_algorithmName, ALGORITHM_NAME_SGBM) == 0 ? STEREO_SGBM :
                              strcmp(g_algorithmName, ALGORITHM_NAME_HH) == 0 ? STEREO_HH :
                              strcmp(g_algorithmName, ALGORITHM_NAME_VAR) == 0 ? STEREO_VAR : STEREO_VALID;
            if (g_algorithm < 0)
            {
                LOGE("Command-line parameter error: Unknown stereo algorithm\n\n");
                return false;
            }
        }
        else if (strncmp(argv[i], MAX_DISPARITY_OPTION, strlen(MAX_DISPARITY_OPTION)) == 0)
        {
            if (sscanf(argv[i] + strlen(MAX_DISPARITY_OPTION), "%d", &g_numDisparities) != 1 ||
                g_numDisparities < 1 || g_numDisparities % 16 != 0)
            {
                LOGE("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
                return false;
            }
        }
        else if (strncmp(argv[i], BLOCK_SIZE_OPTION, strlen(BLOCK_SIZE_OPTION)) == 0)
        {
            if (sscanf(argv[i] + strlen(BLOCK_SIZE_OPTION), "%d", &g_SADWindowSize) != 1 ||
                g_SADWindowSize < 1 || g_SADWindowSize % 2 != 1)
            {
                LOGE("Command-line parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
                return false;
            }
        }
        else if (strncmp(argv[i], SCALE_OPTION, strlen(SCALE_OPTION)) == 0)
        {
            if (sscanf(argv[i] + strlen(SCALE_OPTION), "%f", &g_scale) != 1 || g_scale < 0)
            {
                LOGE("Command-line parameter error: The scale factor (--scale=<...>) must be a positive floating-point number\n");
                return false;
            }
        }
        else if (strcmp(argv[i], "-i") == 0)
        {
            g_intrinsicFileName = argv[++i];
        }
        else if (strcmp(argv[i], "-e") == 0)
        {
            g_extrinsicFileName = argv[++i];
        }
        else
        {
            LOGE("Command-line parameter error: unknown option %s\n", argv[i]);
            return false;
        }
    }

    return true;
}

bool LoadCameraParameters()
{
    if (!g_intrinsicFileName || !g_extrinsicFileName)
        return false;

    // reading intrinsic parameters
    FileStorage fs(g_intrinsicFileName, FileStorage::READ);
    if (!fs.isOpened())
    {
        LOGE("Failed to open file %s\n", g_intrinsicFileName);
        return false;
    }

    Mat M1, D1, M2, D2;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;

    M1 *= g_scale;
    M2 *= g_scale;

    fs.open(g_extrinsicFileName, FileStorage::READ);
    if (!fs.isOpened())
    {
        LOGE("Failed to open file %s\n", g_intrinsicFileName);
        return false;
    }

    Mat R, T;
    fs["R"] >> R;
    fs["T"] >> T;

    if (R.cols == 1 && R.rows == 3)
    {
        Rodrigues(R, R);
    }

    stereoRectify(M1, D1, M2, D2, g_imgSize, R, T, g_R1, g_R2, g_P1, g_P2, g_Q, CALIB_ZERO_DISPARITY, -1, g_imgSize, &g_roi1, &g_roi2);

    initUndistortRectifyMap(M1, D1, g_R1, g_P1, g_imgSize, CV_16SC2, g_map11, g_map12);
    initUndistortRectifyMap(M2, D2, g_R2, g_P2, g_imgSize, CV_16SC2, g_map21, g_map22);

    return true;
}

bool InitAlgorithm(Mat img)
{
    g_numDisparities = g_numDisparities > 0 ? g_numDisparities : ((g_imgSize.width / 8) + 15) & - 16;

    g_bm->setROI1(g_roi1);
    g_bm->setROI2(g_roi2);
    g_bm->setPreFilterCap(31);
    g_bm->setBlockSize(g_SADWindowSize > 0 ? g_SADWindowSize : 9);
    g_bm->setMinDisparity(0);
    g_bm->setNumDisparities(g_numDisparities);
    g_bm->setTextureThreshold(10);
    g_bm->setUniquenessRatio(15);
    g_bm->setSpeckleWindowSize(100);
    g_bm->setSpeckleRange(32);
    g_bm->setDisp12MaxDiff(1);

    g_sgbm->setPreFilterCap(63);
    int sgbmWinSize = g_SADWindowSize > 0 ? g_SADWindowSize : 3;
    g_sgbm->setBlockSize(sgbmWinSize);

    int cn = img.channels();

    g_sgbm->setP1(8 * cn * sgbmWinSize * sgbmWinSize);
    g_sgbm->setP2(32 * cn * sgbmWinSize * sgbmWinSize);
    g_sgbm->setMinDisparity(0);
    g_sgbm->setNumDisparities(g_numDisparities);
    g_sgbm->setUniquenessRatio(10);
    g_sgbm->setSpeckleWindowSize(100);
    g_sgbm->setSpeckleRange(32);
    g_sgbm->setDisp12MaxDiff(1);
    g_sgbm->setMode(g_algorithm == STEREO_HH ? StereoSGBM::MODE_HH : StereoSGBM::MODE_SGBM);

    return true;
}

int main(int argc, char *argv[])
{
    VideoCapture videoCapture1;
    VideoCapture videoCapture2;
    Size         g_imgSize = Size(g_cameraWidth, g_cameraHeight);
    int          i         = 0;

#if SAVE_CAMERA_FRAME
    CvVideoWriter *writer1 = cvCreateVideoWriter("video1.avi", -1, 25, cvSize(g_cameraWidth, g_cameraHeight));
    CvVideoWriter *writer2 = cvCreateVideoWriter("video2.avi", -1, 25, cvSize(g_cameraWidth, g_cameraHeight));
#endif

    if (!ParseCmd(argc, argv))
    {
        return 0;
    }

    if (!LoadCameraParameters())
    {
        return 0;
    }

    videoCapture1.open(0);
    videoCapture2.open(1);

    // Try to set the camera resolution. Note that this only works for some cameras on
    // some computers and only for some drivers, so don't rely on it to work!
    videoCapture1.set(CV_CAP_PROP_FRAME_WIDTH, g_cameraWidth);
    videoCapture1.set(CV_CAP_PROP_FRAME_HEIGHT, g_cameraHeight);
    videoCapture2.set(CV_CAP_PROP_FRAME_WIDTH, g_cameraWidth);
    videoCapture2.set(CV_CAP_PROP_FRAME_HEIGHT, g_cameraHeight);

    // Create a GUI window for display on the screen.
    namedWindow(g_windowName); // Resizable window, might not work on Windows.
    resizeWindow(g_windowName, g_windowWidth, g_windowHeight);

    // Get OpenCV to automatically call my "onMouse()" function when the user clicks in the GUI window.
    setMouseCallback(g_windowName, OnMouse, 0);

    Mat videoFrame;
    videoCapture1 >> videoFrame;
    videoCapture2 >> videoFrame;
    InitAlgorithm(videoFrame);

    while (1)
    {
        Mat  videoFrame1;
        Mat  videoFrame2;
        Rect dstRC;
        Mat  dstROI;

        videoCapture1 >> videoFrame1;
        videoCapture2 >> videoFrame2;
        if (videoFrame1.empty() || videoFrame2.empty())
        {
            LOGE("ERROR: Couldn't grab the next camera frame.\n");
            break;
        }

#if SAVE_CAMERA_FRAME
        IplImage *pTmp1 = &IplImage(videoFrame1);
        IplImage *pTmp2 = &IplImage(videoFrame2);
        cvWriteFrame(writer1, pTmp1);
        cvWriteFrame(writer2, pTmp2);
#endif

        Mat displayFrame = Mat(Size(g_windowWidth, g_windowHeight), CV_8UC3);

        // Get the destination ROI (and make sure it is within the image!).
        dstRC  = Rect(g_border, g_border, g_cameraWidth, g_cameraHeight);
        dstROI = displayFrame(dstRC);
        videoFrame1.copyTo(dstROI);

        // Get the destination ROI (and make sure it is within the image!).
        dstRC  = Rect(g_cameraWidth + g_border * 2, g_border, g_cameraWidth, g_cameraHeight);
        dstROI = displayFrame(dstRC);
        videoFrame2.copyTo(dstROI);

        Mat img1r, img2r;
        remap(videoFrame1, img1r, g_map11, g_map12, INTER_LINEAR);
        remap(videoFrame1, img2r, g_map21, g_map22, INTER_LINEAR);

        videoFrame1 = img1r;
        videoFrame1 = img2r;

        Mat disp, disp8;

        int64 t = getTickCount();

        if (g_algorithm == STEREO_BM)
        {
            g_bm->compute(videoFrame1, videoFrame2, disp);
        }
        else if (g_algorithm == STEREO_SGBM || g_algorithm == STEREO_HH)
        {
            g_sgbm->compute(videoFrame1, videoFrame2, disp);
        }

        t = getTickCount() - t;
        LOGE("#%d---Time elapsed: %fms\n", ++i, t * 1000 / getTickFrequency());

        if (g_algorithm != STEREO_VAR)
            disp.convertTo(disp8, CV_8U, 255 / (g_numDisparities * 16.));
        else
            disp.convertTo(disp8, CV_8U);

        imwrite("hy.jpg", disp8);

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

#if SAVE_CAMERA_FRAME
        else if (keypress == VK_SPACE)
        {
            static int x = 0;
            char       buf[128];

            x++;

            memset(buf, 0, 128);
            sprintf(buf, "videoFrame1_%d.jpg", x);
            cvSaveImage(buf, pTmp1);

            memset(buf, 0, 128);
            sprintf(buf, "videoFrame2_%d.jpg", x);
            cvSaveImage(buf, pTmp2);
        }
#endif
    }

#if SAVE_CAMERA_FRAME
    cvReleaseVideoWriter(&writer1);
    cvReleaseVideoWriter(&writer2);
#endif

    return 0;
}