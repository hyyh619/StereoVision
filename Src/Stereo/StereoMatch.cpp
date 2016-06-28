#include <stdio.h>
#include <string.h>
#include <vector>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include "TqcLog.h"
#include "TqcUtils.h"
#include "Config.h"
#include "StereoVision.h"
#include "StereoCamera.h"
#include "StereoMatchAlgorithm.h"
#include "StereoUtils.h"

using namespace cv;
using namespace std;

int  g_width  = 0;
int  g_height = 0;
Mat  g_disp;
Mat  g_Q;
Size g_imgSize          = Size(320, 240);
Size g_camCalibrateSize = Size(320, 240);

void SaveTimeCost(const char *postfixName, const char *strOutputPath, const char *strAlgorithmName, int64 time);

// Mouse event handler. Called automatically by OpenCV when the user clicks in the GUI window.
void OnMouse(int event, int x, int y, int, void*)
{
    // We only care about left-mouse clicks, not right-mouse clicks or mouse movement.
    if (event != CV_EVENT_LBUTTONDOWN)
        return;

    Point3d p;

    StereoReprojectPixelTo3D(g_disp, g_Q, Point2i(x, y), p);

    LOGE("(%d, %d, %d): %f, %f, %f\n", x, y, g_disp.at<short>(y, x), p.x, p.y, p.z);
}

int main(int argc, char **argv)
{
    vector<char*> fileList1;
    vector<char*> fileList2;

    char  filePre[TQC_MAX_PATH];
    int64 totalTimeCost       = 0;
    int   totalFrame          = 0;

    if (argc < 3 || !ParseCmd(argc, argv, g_option))
    {
        PrintHelp();
        return -1;
    }

    if (!CheckOption(g_option))
    {
        return -1;
    }

    // Add files to file list.
    if (g_option.strLeftFile && g_option.strRightFile)
    {
        fileList1.push_back((char*)g_option.strLeftFile);
        fileList2.push_back((char*)g_option.strRightFile);
    }
    else if (g_option.strLeftPrefix && g_option.strRightPrefix)
    {
        AddFileList(g_option.strLeftPrefix, fileList1);
        AddFileList(g_option.strRightPrefix, fileList2);
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

    // Default channel's number is 3.
    if (!StereoInitAlgorithm(3,
                             g_CamParam.roi1,
                             g_CamParam.roi2,
                             g_option.nNumDisparities,
                             g_option.nSADWindowSize,
                             g_imgSize.width,
                             g_option.algorithm))
    {
        return -1;
    }

    // Loop all files.
    for (int i = 0; i < fileList1.size() && i < fileList2.size(); i++)
    {
        int    nColorMode = (g_option.algorithm == TQC_STEREO_BM ? 0 : -1);
        double d[3][3]    = { 0.0f };
        Mat    img1       = imread(fileList1.at(i), nColorMode);
        Mat    img2       = imread(fileList2.at(i), nColorMode);

        char   path[TQC_MAX_PATH];
        char   *leftFilePre = fileList1.at(i);
        size_t len          = strlen(leftFilePre) - 1;
        size_t orgLen       = len;
        Mat     disp;
        Mat     disp8;

        memset(path, 0, TQC_MAX_PATH);
        memset(filePre, 0, TQC_MAX_PATH);

        while (leftFilePre[len] != '\\')
        {
            len--;
        }

        memcpy(path, leftFilePre, len + 1);
        memcpy(filePre, &leftFilePre[len + 1], orgLen - len);

        if (img1.empty() || img2.empty())
        {
            LOGE("Command-line parameter error: could not load the input image files\n");
            return -1;
        }

        // Begin time record.
        int64 t = getTickCount();

        Size imgSize = img1.size();

        g_width = imgSize.width;
        g_height = imgSize.height;

        if (!StereoMatch(img1, img2, g_option.fScale, g_option.algorithm, g_CamParam, disp))
        {
            LOGE("%s(%d): cannot match left and right images.", __FUNCTION__, __LINE__);
            return -1;
        }

        g_disp = disp;

#if TQC_FILTER_DEPTH_VALUE
        // Filter, if depth > 5m, we will skip this.
        StereoFilterDisp(disp, g_CamParam.Q);
#endif

        disp8 = StereoGetDisp8FromDisp(disp, g_algorithmParam.selector, g_algorithmParam.nNumDisparities);
        StereoCalcDepthOfVirtualCopter(disp, g_CamParam.Q, d);

        // Output time cost.
        t = getTickCount() - t;
        SaveTimeCost(filePre, g_option.strOutputPath, g_option.strAlgorithmName, t);
        if (i != 0)
        {
            totalFrame++;
            totalTimeCost += t;
        }

        // LOGE("****************************************");
        // LOGE("* %08.3f * %08.3f * %08.3f *", d[0][0], d[0][1], d[0][2]);
        // LOGE("* %08.3f * %08.3f * %08.3f *", d[1][0], d[1][1], d[1][2]);
        // LOGE("* %08.3f * %08.3f * %08.3f *", d[2][0], d[2][1], d[2][2]);
        // LOGE("****************************************");

#if TQC_OUTPUT_VIRTUAL_COPTER_DEPTH_TO_FILE
        if (g_option.depthFile)
        {
            char *strFileName = GetFileName("disp", filePre, "jpg", g_option.strOutputPath, g_option.strAlgorithmName, imgSize.width, imgSize.height);
            fprintf(g_option.depthFile, "%s\n", strFileName);
            fprintf(g_option.depthFile, "****************************************\n");
            fprintf(g_option.depthFile, "* %08.3f * %08.3f * %08.3f *\n", d[0][0], d[0][1], d[0][2]);
            fprintf(g_option.depthFile, "* %08.3f * %08.3f * %08.3f *\n", d[1][0], d[1][1], d[1][2]);
            fprintf(g_option.depthFile, "* %08.3f * %08.3f * %08.3f *\n", d[2][0], d[2][1], d[2][2]);
            fprintf(g_option.depthFile, "****************************************\n\n");
        }
#endif

#if TQC_OUTPUT_DISP_VALUE_TO_FILE
        // benet-add for matlab display
        if (g_option.strDispFile)
        {
            // LOGE("Q Matrix: 0x%X\n", Q.type());
            // LOGE("%f %f %f %f\n", Q.at<double>(0, 0), Q.at<double>(0, 1), Q.at<double>(0, 2), Q.at<double>(0, 3));
            // LOGE("%f %f %f %f\n", Q.at<double>(1, 0), Q.at<double>(1, 1), Q.at<double>(1, 2), Q.at<double>(1, 3));
            // LOGE("%f %f %f %f\n", Q.at<double>(2, 0), Q.at<double>(2, 1), Q.at<double>(2, 2), Q.at<double>(2, 3));
            // LOGE("%f %f %f %f\n", Q.at<double>(3, 0), Q.at<double>(3, 1), Q.at<double>(3, 2), Q.at<double>(3, 3));
            SaveDispData(g_option.strDispFile, filePre, g_option.strOutputPath, g_option.strAlgorithmName, disp);
        }
#endif

        if (g_option.bDisplay)
        {
            namedWindow("left", 1);
            imshow("left", img1);
            namedWindow("right", 1);
            imshow("right", img2);
            namedWindow("disparity", 0);
            imshow("disparity", disp8);

            // Get OpenCV to automatically call my "onMouse()" function when the user clicks in the GUI window.
            setMouseCallback("disparity", OnMouse, 0);

            LOGE("press any key to continue...");
            fflush(stdout);
            waitKey();
            LOGE("\n");
        }

#if TQC_OUTPUT_3D_PCL_TO_FILE
        if (g_option.strPCLFile)
        {
            LOGE("storing the point cloud...");
            fflush(stdout);
            Mat xyz;
            reprojectImageTo3D(disp, xyz, g_CamParam.Q, true);
            SaveXYZData(g_option.strPCLFile, filePre, g_option.strOutputPath, g_option.strAlgorithmName, xyz);
            LOGE("\n");
        }
#endif

#if TQC_OUTPUT_DISP_TO_IMAGE
        SavePic(filePre, g_option.strOutputPath, g_option.strAlgorithmName, disp8);
#endif
    }

    SaveTimeCost(filePre, g_option.strOutputPath, g_option.strAlgorithmName, totalTimeCost / totalFrame);

    for (int i = 0; i < fileList1.size(); i++)
    {
        char *p = fileList1.at(i);
        if (p)
        {
            free(p);
        }
    }

    for (int i = 0; i < fileList2.size(); i++)
    {
        char *p = fileList2.at(i);
        if (p)
        {
            free(p);
        }
    }

    fileList1.clear();
    fileList2.clear();

    return 0;
}

void SaveTimeCost(const char *postfixName, const char *strOutputPath, const char *strAlgorithmName, int64 time)
{
    static int i = 0;
    char       fileName[TQC_MAX_PATH];
    char       *strPicName = NULL;
    FILE       *fp         = NULL;
    float      fTime       = 0.0f;

    fTime = (float)(time * 1000 / getTickFrequency());
    LOGE("#%d---Time elapsed: %8.3fms\n", ++i, fTime);

    memset(fileName, 0, TQC_MAX_PATH);
    sprintf(fileName, "%s/time_cost.log", g_option.strOutputPath);

    fp = fopen(fileName, "a+");
    if (!fp)
    {
        LOGE("%s(%d): cannot open file %s", __FUNCTION__, __LINE__, fileName);
        return;
    }

    strPicName = GetFileName("disp", postfixName, "jpg", strOutputPath, strAlgorithmName, g_width, g_height);
    fprintf(fp, "%s: %8.3fms\n", strPicName, fTime);
}