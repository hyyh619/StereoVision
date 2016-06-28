#ifndef __STEREO_UTILS_H
#define __STEREO_UTILS_H

#include <stdio.h>
#include "Config.h"
#include "StereoMatchAlgorithm.h"

#define TQC_ALGORITHM_OPTION    "--algorithm="
#define TQC_ALGORITHM_NAME_BM   "bm"
#define TQC_ALGORITHM_NAME_SGBM "sgbm"
#define TQC_ALGORITHM_NAME_HH   "hh"
#define TQC_ALGORITHM_NAME_VAR  "var"

#define TQC_MAX_DISPARITY_OPTION "--max-disparity="
#define TQC_BLOCK_SIZE_OPTION    "--blocksize="
#define TQC_SCALE_OPTION         "--scale="
#define TQC_NO_DISPLAY_OPTION    "--no-display"

typedef struct _stCmdOption
{
    char        *strAlgorithmName;
    float       fScale;
    enAlgorithm algorithm;
    int         nSADWindowSize;
    int         nNumDisparities;
    bool        bDisplay;

    char *strIntrinsicFile = 0;
    char *strExtrinsicFile = 0;
    char *strLeftFile;
    char *strRightFile;
    char *strDispFile;
    char *strPCLFile;
    char *strDepthFile;
    char *strOutputPath;
    FILE *depthFile;
    char *strLeftPrefix;
    char *strRightPrefix;

    void stCmdOption()
    {
        bDisplay         = true;
        strAlgorithmName = NULL;
        strIntrinsicFile = NULL;
        strExtrinsicFile = NULL;
        fScale           = TQC_IMAGE_SCALE;
        algorithm        = TQC_STEREO_SGBM;
        nSADWindowSize   = TQC_SAD_WINDOW_SIZE;
        nNumDisparities  = TQC_NUM_DISPARITIES;

        strLeftFile    = NULL;
        strRightFile   = NULL;
        strDispFile    = NULL;
        strPCLFile     = NULL;
        strDepthFile   = NULL;
        strOutputPath  = NULL;
        depthFile      = NULL;
        strLeftPrefix  = NULL;
        strRightPrefix = NULL;
    }
} stCmdOption;

// Function declaration
bool ParseCmd(int argc, char *argv[], stCmdOption &cmd);
void PrintHelp();
bool CheckOption(stCmdOption option);
bool GenerateMipmap(Mat img1, Mat img2, int width, int height, const char *filePreLeft, const char *filePreRight);
void SavePic(const char *postfixName, const char *strOutputPath, const char *strAlgorithmName, Mat &disp8);
void SaveDispData(const char *filename, const char *postfixName, const char *strOutputPath, const char *strAlgorithmName, const Mat &mat);
void SaveXYZData(const char *filename, const char *postfixName, const char *strOutputPath, const char *strAlgorithmName, const Mat &mat);
char* GetFileName(const char *fileName,
                  const char *postfixName,
                  const char *extName,
                  const char *strOutputPath,
                  const char *strAlgorithmName,
                  int width,
                  int height);
void StereoReprojectPixelTo3D(const Mat disp, const Mat &Q, const Point2i &pixel, Point3d &point);

// Global variables declaration
extern stCmdOption g_option;

#endif /* __STEREO_UTILS_H */