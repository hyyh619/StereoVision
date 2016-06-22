#include <stdio.h>
#include <string.h>
#include <vector>
#include <windows.h>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include "TqcLog.h"
#include "TqcUtils.h"
#include "Config.h"

using namespace cv;
using namespace std;

const char *g_windowName = "StereoPhoto";      // Name shown in the GUI window.
int        g_cameraWidth = 320;
int        g_cameraHeight = 240;
int        g_border = 5;
int        g_windowWidth = g_cameraWidth * 2 + g_border * 3;
int        g_windowHeight = g_cameraHeight + g_border * 2;
Mat        g_videoFrame1;
Mat        g_videoFrame2;

void SaveStereoPictures()
{
    static int counter = 0;
    char buf[TQC_MAX_PATH];

    memset(buf, 0, TQC_MAX_PATH);
    sprintf_s(buf, "videoLeft_%04d.jpg", counter);
    imwrite(buf, g_videoFrame1);
    LOGE("Save image: %s", buf);

    memset(buf, 0, TQC_MAX_PATH);
    sprintf_s(buf, "videoRight_%04d.jpg", counter);
    imwrite(buf, g_videoFrame2);
    LOGE("Save image: %s", buf);

    counter++;
}

// Mouse event handler. Called automatically by OpenCV when the user clicks in the GUI window.
void OnMouse(int event, int x, int y, int, void*)
{
    // We only care about left-mouse clicks, not right-mouse clicks or mouse movement.
    if (event != CV_EVENT_LBUTTONDOWN)
        return;

    SaveStereoPictures();
}

bool OnKeyboard()
{
    // IMPORTANT: Wait for at least 20 milliseconds, so that the image can be displayed on the screen!
    // Also checks if a key was pressed in the GUI window. Note that it should be a "char" to support Linux.
    char keypress = waitKey(20);  // This is needed if you want to see anything!

    switch (keypress)
    {
    case VK_RETURN:
        SaveStereoPictures();
        break;

    case VK_ESCAPE:
        return true;

    default:
        break;
    }

    return false;
}

int main(int argc, char **argv)
{
    VideoCapture videoCapture1;
    VideoCapture videoCapture2;

    videoCapture1.open(TQC_LOGICAL_CAM_LEFT_INDEX);
    videoCapture2.open(TQC_LOGICAL_CAM_RIGHT_INDEX);

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

    while (1)
    {
        Rect dstRC;
        Mat  dstROI;
        Mat displayFrame = Mat(Size(g_windowWidth, g_windowHeight), CV_8UC3);

        videoCapture1 >> g_videoFrame1;
        videoCapture2 >> g_videoFrame2;
        if (g_videoFrame1.empty() || g_videoFrame2.empty())
        {
            LOGE("ERROR: Couldn't grab the next camera frame.\n");
            break;
        }

        // Get the destination ROI (and make sure it is within the image!).
        dstRC = Rect(g_border, g_border, g_cameraWidth, g_cameraHeight);
        dstROI = displayFrame(dstRC);
        g_videoFrame1.copyTo(dstROI);

        // Get the destination ROI (and make sure it is within the image!).
        dstRC = Rect(g_cameraWidth + g_border * 2, g_border, g_cameraWidth, g_cameraHeight);
        dstROI = displayFrame(dstRC);
        g_videoFrame2.copyTo(dstROI);

        imshow(g_windowName, displayFrame);

        if (OnKeyboard())
            break;
    }
}