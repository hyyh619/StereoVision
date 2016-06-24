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

using namespace cv;
using namespace std;

char *g_algorithmName = NULL;
char *g_outputPath    = NULL;
int  g_width          = 0;
int  g_height         = 0;
Mat  g_disp;
Mat  g_Q;
Size g_cameraCalibSize = Size(320, 240);
FILE *g_depthFile      = 0;

void SaveDispData(const char *filename, const char *postfixName, const Mat &mat);
void SaveXYZData(const char *filename, const char *postfixName, const Mat &mat);
void SavePic(const char *postfixName, Mat &disp8);
void SaveTimeCost(const char *postfixName, int64 time);
void Gray2Color(CvMat *pGrayMat, CvMat *pColorMat);
void FilterDisp(Mat &disp8);
char* GetFileName(const char *fileName, const char *postfixName, const char *extName);

static void PrintHelp()
{
    LOGE("\nDemo stereo matching converting L and R images into disparity and point clouds\n");
    LOGE("\nUsage: stereo_match <left_image> <right_image> [--algorithm=bm|sgbm|hh] [--blocksize=<block_size>]\n"
         "[--max-disparity=<max_disparity>] [--scale=scale_factor>] [-i <intrinsic_filename>] [-e <extrinsic_filename>]\n"
         "[--no-display] [-o <disparity_image>] [-p <point_cloud_file>]\n"
         "[--path outputPath] [--left left] [--right right]");
}

void ReprojectPixelTo3D(const Mat disp, const Mat &Q, const Point2i &pixel, Point3d &point)
{
    double q[4][4];
    Mat    _Q(4, 4, CV_64F, q);
    int    x = pixel.x;
    int    y = pixel.y;

    Q.convertTo(_Q, CV_64F);

    point.x = ((x + q[0][3]) / (q[3][2] * g_disp.at<short>(y, x) + q[3][3])) * 16;
    point.y = ((y + q[1][3]) / (q[3][2] * g_disp.at<short>(y, x) + q[3][3])) * 16;
    point.z = ((q[2][3]) / (q[3][2] * g_disp.at<short>(y, x) + q[3][3])) * 16;
}

double GetDepthFromPixel(const Mat disp, const Mat &Q, const Point2i &pixel)
{
    double depth;
    double q[4][4];
    Mat    _Q(4, 4, CV_64F, q);
    int    x = pixel.x;
    int    y = pixel.y;

    Q.convertTo(_Q, CV_64F);

    depth = ((q[2][3]) / (q[3][2] * disp.at<short>(y, x) + q[3][3])) * 16;

    return depth;
}

void CalcDepthOfVirtualCopter(const Mat &disp, const Mat &Q, double d[3][3])
{
    for (int j = 0; j < TQC_VIRTUAL_COPTER_Y_SPLITE; j++)
    {
        for (int i = 0; i < TQC_VIRTUAL_COPTER_X_SPLITE; i++)
        {
            int    left   = TQC_VIRTUAL_COPTER_LEFT + TQC_VIRTUAL_COPTER_SUB_X * i;
            int    top    = TQC_VIRTUAL_COPTER_TOP + TQC_VIRTUAL_COPTER_SUB_Y * j;
            int    right  = left + TQC_VIRTUAL_COPTER_SUB_X;
            int    bottom = top + TQC_VIRTUAL_COPTER_SUB_Y;
            double dMin   = 10000.0f;

            for (int y = top; y < bottom; y++)
            {
                for (int x = left; x < right; x++)
                {
                    double cur = GetDepthFromPixel(disp, Q, Point2i(x, y));
                    if (dMin > cur && cur > FLT_EPSILON)
                        dMin = cur;
                }
            }

            d[j][i] = dMin;
        }
    }
}

// Mouse event handler. Called automatically by OpenCV when the user clicks in the GUI window.
void OnMouse(int event, int x, int y, int, void*)
{
    // We only care about left-mouse clicks, not right-mouse clicks or mouse movement.
    if (event != CV_EVENT_LBUTTONDOWN)
        return;

    Point3d p;

    ReprojectPixelTo3D(g_disp, g_Q, Point2i(x, y), p);

    LOGE("(%d, %d, %d): %f, %f, %f\n", x, y, g_disp.at<short>(y, x), p.x, p.y, p.z);
}

int main(int argc, char **argv)
{
    const char *strLeftFile      = NULL;
    const char *strRightFile     = NULL;
    const char *strIntrinsicFile = NULL;
    const char *strExtrinsicFile = NULL;
    const char *strDispFile      = NULL;
    const char *strPCLFile       = NULL;
    const char *strDepthFile     = NULL;
    const char *strLeftPrefix    = NULL;
    const char *strRightPrefix   = NULL;

    vector<char*> fileList1;
    vector<char*> fileList2;

    char  filePre[TQC_MAX_PATH];
    int   alg                 = STEREO_SGBM;
    int   SADWindowSize       = 0;
    int   numberOfDisparities = 0;
    bool  no_display          = false;
    float scale               = 1.f;
    int64 totalTimeCost       = 0;
    int   totalFrame          = 0;

    Ptr<StereoBM>   bm   = StereoBM::create(16, 9);
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);

    if (argc < 3)
    {
        PrintHelp();
        return 0;
    }

    for (int i = 1; i < argc; i++)
    {
        if (argv[i][0] != '-')
        {
            if (!strLeftFile)
                strLeftFile = argv[i];
            else
                strRightFile = argv[i];
        }
        else if (strncmp(argv[i], ALGORITHM_OPTION, strlen(ALGORITHM_OPTION)) == 0)
        {
            g_algorithmName = argv[i] + strlen(ALGORITHM_OPTION);
            alg             = strcmp(g_algorithmName, ALGORITHM_NAME_BM) == 0 ? STEREO_BM :
                              strcmp(g_algorithmName, ALGORITHM_NAME_SGBM) == 0 ? STEREO_SGBM :
                              strcmp(g_algorithmName, ALGORITHM_NAME_HH) == 0 ? STEREO_HH :
                              strcmp(g_algorithmName, ALGORITHM_NAME_VAR) == 0 ? STEREO_VAR : STEREO_VALID;
            if (alg < 0)
            {
                LOGE("Command-line parameter error: Unknown stereo algorithm\n\n");
                PrintHelp();
                return -1;
            }
        }
        else if (strncmp(argv[i], MAX_DISPARITY_OPTION, strlen(MAX_DISPARITY_OPTION)) == 0)
        {
            if (sscanf(argv[i] + strlen(MAX_DISPARITY_OPTION), "%d", &numberOfDisparities) != 1 ||
                numberOfDisparities < 1 || numberOfDisparities % 16 != 0)
            {
                LOGE("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
                PrintHelp();
                return -1;
            }
        }
        else if (strncmp(argv[i], BLOCK_SIZE_OPTION, strlen(BLOCK_SIZE_OPTION)) == 0)
        {
            if (sscanf(argv[i] + strlen(BLOCK_SIZE_OPTION), "%d", &SADWindowSize) != 1 ||
                SADWindowSize < 1 || SADWindowSize % 2 != 1)
            {
                LOGE("Command-line parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
                return -1;
            }
        }
        else if (strncmp(argv[i], SCALE_OPTION, strlen(SCALE_OPTION)) == 0)
        {
            if (sscanf(argv[i] + strlen(SCALE_OPTION), "%f", &scale) != 1 || scale < 0)
            {
                LOGE("Command-line parameter error: The scale factor (--scale=<...>) must be a positive floating-point number\n");
                return -1;
            }
        }
        else if (strcmp(argv[i], NO_DISPLAY_OPTION) == 0)
        {
            no_display = true;
        }
        else if (strcmp(argv[i], "-i") == 0)
        {
            strIntrinsicFile = argv[++i];
        }
        else if (strcmp(argv[i], "-e") == 0)
        {
            strExtrinsicFile = argv[++i];
        }
        else if (strcmp(argv[i], "-o") == 0)
        {
            strDispFile = argv[++i];
        }
        else if (strcmp(argv[i], "-p") == 0)
        {
            strPCLFile = argv[++i];
        }
        else if (strcmp(argv[i], "-v") == 0)
        {
            strDepthFile = argv[++i];
            if (strDepthFile)
            {
                char buf[TQC_MAX_PATH];

                memset(buf, 0, TQC_MAX_PATH);
                sprintf(buf, "%s/%s", g_outputPath, strDepthFile);

                g_depthFile = fopen(buf, "w");
                if (!g_depthFile)
                {
                    LOGE("Cannot open depth file(%s).", buf);
                    waitKey();
                    return -1;
                }
            }
        }
        else if (strcmp(argv[i], "--left") == 0)
        {
            strLeftPrefix = argv[++i];
        }
        else if (strcmp(argv[i], "--right") == 0)
        {
            strRightPrefix = argv[++i];
        }
        else if (strcmp(argv[i], "--path") == 0)
        {
            g_outputPath = argv[++i];
        }
        else
        {
            LOGE("Command-line parameter error: unknown option %s\n", argv[i]);
            return -1;
        }
    }

    if ((!strLeftFile || !strRightFile) &&
        (!strLeftPrefix || !strRightPrefix))
    {
        LOGE("Command-line parameter error: both left and right images must be specified\n");
        return -1;
    }

    if ((strIntrinsicFile != 0) ^ (strExtrinsicFile != 0))
    {
        LOGE("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
        return -1;
    }

    if (strExtrinsicFile == 0 && strPCLFile && g_outputPath)
    {
        LOGE("Command-line parameter error: extrinsic and intrinsic parameters must be specified to compute the point cloud\n");
        return -1;
    }

    if (strLeftFile && strRightFile)
    {
        fileList1.push_back((char*)strLeftFile);
        fileList2.push_back((char*)strRightFile);
    }
    else if (strLeftPrefix && strRightPrefix)
    {
        AddFileList(strLeftPrefix, fileList1);
        AddFileList(strRightPrefix, fileList2);
    }

    for (int i = 0; i < fileList1.size() && i < fileList2.size(); i++)
    {
        int    color_mode = alg == STEREO_BM ? 0 : -1;
        double d[3][3]    = { 0.0f };
        Mat    img1       = imread(fileList1.at(i), color_mode);
        Mat    img2       = imread(fileList2.at(i), color_mode);

        char   path[TQC_MAX_PATH];
        char   *leftFilePre = fileList1.at(i);
        size_t len          = strlen(leftFilePre) - 1;
        size_t orgLen       = len;

        memset(path, 0, TQC_MAX_PATH);
        memset(filePre, 0, TQC_MAX_PATH);

        while (leftFilePre[len] != '\\')
        {
            len--;
        }

        memcpy(path, leftFilePre, len + 1);
        memcpy(filePre, &leftFilePre[len + 1], orgLen - len);

        if (img1.empty())
        {
            LOGE("Command-line parameter error: could not load the first input image file\n");
            return -1;
        }

        if (img2.empty())
        {
            LOGE("Command-line parameter error: could not load the second input image file\n");
            return -1;
        }

        // Begin time record.
        int64 t = getTickCount();

        if (scale != 1.f)
        {
            Mat temp1, temp2;
            int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
            resize(img1, temp1, Size(), scale, scale, method);
            img1 = temp1;
            resize(img2, temp2, Size(), scale, scale, method);
            img2 = temp2;
        }

        Size imgSize = img1.size();

        g_width  = imgSize.width;
        g_height = imgSize.height;

        if (0)
        {
            int  k      = 1;
            Mat  temp1  = img1;
            Mat  temp2  = img2;
            int  width  = g_width;
            int  height = g_height;
            char buf[TQC_MAX_PATH];

            char   filePreRight[TQC_MAX_PATH];
            char   *rightFilePre = fileList2.at(i);
            size_t len           = strlen(rightFilePre) - 1;
            size_t orgLen        = len;

            memset(path, 0, TQC_MAX_PATH);
            memset(filePreRight, 0, TQC_MAX_PATH);

            while (leftFilePre[len] != '\\')
            {
                len--;
            }

            memcpy(path, leftFilePre, len + 1);
            memcpy(filePreRight, &rightFilePre[len + 1], orgLen - len);

            do
            {
                Mat tmpScale1;
                Mat tmpScale2;

                width  /= 2;
                height /= 2;

                resize(temp1, tmpScale1, Size(), 0.5f, 0.5f, INTER_LINEAR);
                resize(temp2, tmpScale2, Size(), 0.5f, 0.5f, INTER_LINEAR);

                memset(buf, 0, 256);
                sprintf(buf, "%s/%s_%d.jpg", g_outputPath, filePre, 20 + k);
                imwrite(buf, temp1);

                memset(buf, 0, 256);
                sprintf(buf, "%s/%s_%d.jpg", g_outputPath, filePreRight, 20 + k);
                imwrite(buf, temp2);

                k++;

                temp1 = tmpScale1;
                temp2 = tmpScale2;
            }
            while (width > 32 && height > 32);
        }

        Rect roi1, roi2;
        Mat  Q;

        if (strIntrinsicFile)
        {
            // reading intrinsic parameters
            FileStorage fs(strIntrinsicFile, FileStorage::READ);
            if (!fs.isOpened())
            {
                LOGE("Failed to open file %s\n", strIntrinsicFile);
                return -1;
            }

            Mat M1, D1, M2, D2;
            fs["M1"] >> M1;
            fs["D1"] >> D1;
            fs["M2"] >> M2;
            fs["D2"] >> D2;

            M1 *= scale;
            M2 *= scale;

            if (imgSize != g_cameraCalibSize)
            {
                double sx = (double)imgSize.width / g_cameraCalibSize.width;
                double sy = (double)imgSize.height / g_cameraCalibSize.height;

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
                LOGE("Failed to open file %s\n", strExtrinsicFile);
                return -1;
            }

            Mat R, T, R1, P1, R2, P2;
            fs["R"] >> R;
            fs["T"] >> T;

            if (R.cols == 1 && R.rows == 3)
            {
                Rodrigues(R, R);
            }

            Size calibrateImgSize = Size(640, 480);
            stereoRectify(M1, D1, M2, D2, imgSize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, imgSize, &roi1, &roi2);

            Mat map11, map12, map21, map22;
            initUndistortRectifyMap(M1, D1, R1, P1, imgSize, CV_16SC2, map11, map12);
            initUndistortRectifyMap(M2, D2, R2, P2, imgSize, CV_16SC2, map21, map22);

            Mat img1r, img2r;
            remap(img1, img1r, map11, map12, INTER_LINEAR);
            remap(img2, img2r, map21, map22, INTER_LINEAR);

            img1 = img1r;
            img2 = img2r;

            g_Q = Q;
        }

        numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((imgSize.width / 8) + 15) & - 16;

        bm->setROI1(roi1);
        bm->setROI2(roi2);
        bm->setPreFilterCap(31);
        bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
        bm->setMinDisparity(-16);
        bm->setNumDisparities(numberOfDisparities);
        bm->setTextureThreshold(10);
        bm->setUniquenessRatio(15);
        bm->setSpeckleWindowSize(100);
        bm->setSpeckleRange(32);
        bm->setDisp12MaxDiff(1);

        sgbm->setPreFilterCap(63);
        int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
        sgbm->setBlockSize(sgbmWinSize);

        int cn = img1.channels();

        sgbm->setP1(8 * cn * sgbmWinSize * sgbmWinSize);
        sgbm->setP2(32 * cn * sgbmWinSize * sgbmWinSize);
        sgbm->setMinDisparity(0);
        sgbm->setNumDisparities(numberOfDisparities);
        sgbm->setUniquenessRatio(10);
        sgbm->setSpeckleWindowSize(100);
        sgbm->setSpeckleRange(32);
        sgbm->setDisp12MaxDiff(1);
        sgbm->setMode(alg == STEREO_HH ? StereoSGBM::MODE_HH : StereoSGBM::MODE_SGBM);

#if TQC_STEREO_CULL
        {
            Rect dstRC;
            Mat  dstROI;

            // Get the destination ROI (and make sure it is within the image!).
            dstRC = Rect(TQC_STEREO_CAMERA_X_BORDER, TQC_STEREO_CAMERA_Y_BORDER,
                         img1.cols - TQC_STEREO_CAMERA_X_BORDER * 2, img2.rows - TQC_STEREO_CAMERA_Y_BORDER * 2);
            img1 = img1(dstRC);
            img2 = img2(dstRC);
        }
#endif

        Mat disp;
        Mat disp8;

        if (alg == STEREO_BM)
        {
            bm->compute(img1, img2, disp);
        }
        else if (alg == STEREO_SGBM || alg == STEREO_HH)
        {
            sgbm->compute(img1, img2, disp);
        }

        g_disp = disp;

        // Filter, if depth > 5m, we will skip this.
        // benet: disable Filter disparity value.
        // FilterDisp(disp);

        // disp = dispp.colRange(numberOfDisparities, img1p.cols);
        if (alg != STEREO_VAR)
            disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities * 16.));
        else
            disp.convertTo(disp8, CV_8U);

        CalcDepthOfVirtualCopter(disp, Q, d);

        // Output time cost.
        t = getTickCount() - t;
        SaveTimeCost(filePre, t);
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
        if (g_depthFile)
        {
            char *strFileName = GetFileName("disp", filePre, "jpg");
            fprintf(g_depthFile, "%s\n", strFileName);
            fprintf(g_depthFile, "****************************************\n");
            fprintf(g_depthFile, "* %08.3f * %08.3f * %08.3f *\n", d[0][0], d[0][1], d[0][2]);
            fprintf(g_depthFile, "* %08.3f * %08.3f * %08.3f *\n", d[1][0], d[1][1], d[1][2]);
            fprintf(g_depthFile, "* %08.3f * %08.3f * %08.3f *\n", d[2][0], d[2][1], d[2][2]);
            fprintf(g_depthFile, "****************************************\n\n");
        }
#endif

#if TQC_OUTPUT_DISP_VALUE_TO_FILE
        // benet-add for matlab display
        if (strDispFile)
        {
            // LOGE("Q Matrix: 0x%X\n", Q.type());
            // LOGE("%f %f %f %f\n", Q.at<double>(0, 0), Q.at<double>(0, 1), Q.at<double>(0, 2), Q.at<double>(0, 3));
            // LOGE("%f %f %f %f\n", Q.at<double>(1, 0), Q.at<double>(1, 1), Q.at<double>(1, 2), Q.at<double>(1, 3));
            // LOGE("%f %f %f %f\n", Q.at<double>(2, 0), Q.at<double>(2, 1), Q.at<double>(2, 2), Q.at<double>(2, 3));
            // LOGE("%f %f %f %f\n", Q.at<double>(3, 0), Q.at<double>(3, 1), Q.at<double>(3, 2), Q.at<double>(3, 3));
            SaveDispData(strDispFile, filePre, disp);
        }
#endif

        if (!no_display)
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
        if (strPCLFile)
        {
            LOGE("storing the point cloud...");
            fflush(stdout);
            Mat xyz;
            reprojectImageTo3D(disp, xyz, Q, true);
            SaveXYZData(strPCLFile, filePre, xyz);
            LOGE("\n");
        }
#endif

#if TQC_OUTPUT_DISP_TO_IMAGE
        SavePic(filePre, disp8);
#endif
    }

    SaveTimeCost(filePre, totalTimeCost / totalFrame);

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

char* GetFileName(const char *fileName, const char *postfixName, const char *extName)
{
    static char output[TQC_MAX_PATH];

    memset(output, 0, TQC_MAX_PATH);
    sprintf(output, "%s/%s_%s_%dx%d_%s.%s", g_outputPath, fileName, g_algorithmName, g_width, g_height, postfixName, extName);

    return output;
}

void SavePic(const char *postfixName, Mat &disp8)
{
    char *strFileName;

    // Save disparity picture
    strFileName = GetFileName("disp", postfixName, "jpg");
    imwrite(strFileName, disp8);

    // Save color picture
    CvMat *pColorMat = cvCreateMat(disp8.rows, disp8.cols, CV_8UC3);
    CvMat grayMat    = disp8;
    Gray2Color(&grayMat, pColorMat);
    strFileName = GetFileName("color", postfixName, "jpg");
    Mat colorMat1 = Mat(pColorMat->rows, pColorMat->cols, CV_8UC3, pColorMat->data.ptr);
    imwrite(strFileName, colorMat1);
    cvReleaseMat(&pColorMat);
}

void SaveDispData(const char *filename, const char *postfixName, const Mat &mat)
{
    char *buf;
    FILE *fp = NULL;

    buf = GetFileName(filename, postfixName, "dat");
    fp  = fopen(buf, "wt");

    fprintf(fp, "%02d\n", mat.rows);
    fprintf(fp, "%02d\n", mat.cols);

    for (int y = 0; y < mat.rows; y++)
    {
        for (int x = 0; x < mat.cols; x++)
        {
            int disp = (int)mat.at<short>(y, x);    // 这里视差矩阵是CV_16S 格式的，故用 short 类型读取
            fprintf(fp, "%d %d %d\n", x, y, disp);  // 若视差矩阵是 CV_32F 格式，则用 float 类型读取
        }
    }

    fclose(fp);
}

void SaveXYZData(const char *filename, const char *postfixName, const Mat &mat)
{
#if TQC_SAVE_XYZ_FILTER
    const double max_z = 1.0e4;
#endif

    char *buf;
    FILE *fp = NULL;

    buf = GetFileName(filename, postfixName, "dat");
    fp  = fopen(buf, "wt");

    for (int y = 0; y < mat.rows; y++)
    {
        for (int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);

#if TQC_SAVE_XYZ_FILTER
            if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z)
                continue;
#endif

            // fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
            fprintf(fp, "%f %f %f\n", point[0] * 16, point[1] * 16, point[2] * 16);
        }
    }

    fclose(fp);
}

void Gray2Color(CvMat *pGrayMat, CvMat *pColorMat)
{
    if (pColorMat)
        cvZero(pColorMat);

    int stype = CV_MAT_TYPE(pGrayMat->type), dtype = CV_MAT_TYPE(pColorMat->type);
    int rows  = pGrayMat->rows;
    int cols  = pGrayMat->cols;

    // 判断输入的灰度图和输出的伪彩色图是否大小相同、格式是否符合要求
    if (CV_ARE_SIZES_EQ(pGrayMat, pColorMat) && stype == CV_8UC1 && dtype == CV_8UC3)
    {
        CvMat *red   = cvCreateMat(rows, cols, CV_8U);
        CvMat *green = cvCreateMat(rows, cols, CV_8U);
        CvMat *blue  = cvCreateMat(rows, cols, CV_8U);
        CvMat *mask  = cvCreateMat(rows, cols, CV_8U);

        // 计算各彩色通道的像素值
        cvSubRS(pGrayMat, cvScalar(255), blue); // blue(I) = 255 - gray(I)
        cvCopy(pGrayMat, red);                  // red(I) = gray(I)
        cvCopy(pGrayMat, green);                // green(I) = gray(I),if gray(I) < 128
        cvCmpS(green, 128, mask, CV_CMP_GE);    // green(I) = 255 - gray(I), if gray(I) >= 128
        cvSubRS(green, cvScalar(255), green, mask);
        cvConvertScale(green, green, 2.0, 0.0);

        // 合成伪彩色图
        cvMerge(blue, green, red, NULL, pColorMat);

        cvReleaseMat(&red);
        cvReleaseMat(&green);
        cvReleaseMat(&blue);
        cvReleaseMat(&mask);
    }
}

void SaveTimeCost(const char *postfixName, int64 time)
{
    static int i = 0;
    char       fileName[TQC_MAX_PATH];
    char       *strPicName = NULL;
    FILE       *fp         = NULL;
    float      fTime       = 0.0f;

    fTime = (float)(time * 1000 / getTickFrequency());
    LOGE("#%d---Time elapsed: %8.3fms\n", ++i, fTime);

    memset(fileName, 0, TQC_MAX_PATH);
    sprintf(fileName, "%s/time_cost.log", g_outputPath);

    fp = fopen(fileName, "a+");
    if (!fp)
    {
        LOGE("%s(%d): cannot open file %s", __FUNCTION__, __LINE__, fileName);
        return;
    }

    strPicName = GetFileName("disp", postfixName, "jpg");
    fprintf(fp, "%s: %8.3fms\n", strPicName, fTime);
}

void FilterDisp(Mat &disp)
{
    double q[4][4];
    Mat    _Q(4, 4, CV_64F, q);

    g_Q.convertTo(_Q, CV_64F);

    for (int y = 0; y < disp.rows; y++)
    {
        for (int x = 0; x < disp.cols; x++)
        {
            int    disp8 = (int)disp.at<short>(y, x);
            double zc    = ((q[2][3]) / (q[3][2] * disp8 + q[3][3])) * 16;

            // Filter, if depth > 5m, we will skip this.
            if (zc > 5000.0f)
            {
                disp.at<short>(y, x) = -16;
            }
        }
    }
}