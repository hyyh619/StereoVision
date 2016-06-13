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
#include "StereoVision.h"

using namespace cv;
using namespace std;

#define SAVE_XYZ_FILTER 1

char *g_algorithmName = NULL;
char *g_outputPath    = NULL;
int  g_width          = 0;
int  g_height         = 0;
Mat  g_disp;
Mat  g_Q;

void SaveDispData(const char *filename, const char *postfixName, const Mat &mat);
void SaveXYZData(const char *filename, const char *postfixName, const Mat &mat);
void SavePic(const char *postfixName, Mat &disp8);
void SaveTimeCost(const char *postfixName, int64 time);
void Gray2Color(CvMat *pGrayMat, CvMat *pColorMat);

static void PrintHelp()
{
    LOGE("\nDemo stereo matching converting L and R images into disparity and point clouds\n");
    LOGE("\nUsage: stereo_match <left_image> <right_image> [--algorithm=bm|sgbm|hh] [--blocksize=<block_size>]\n"
         "[--max-disparity=<max_disparity>] [--scale=scale_factor>] [-i <intrinsic_filename>] [-e <extrinsic_filename>]\n"
         "[--no-display] [-o <disparity_image>] [-p <point_cloud_file>]\n"
         "[--path outputPath] [--left left] [--right right]");
}

void AddFileList(const char *lpPath, const char *filePrefix, vector<char*> &fileList)
{
    char            szFind[TQC_MAX_PATH];
    WIN32_FIND_DATA FindFileData;

    strcpy(szFind, lpPath);
    strcat(szFind, "*.*");

    HANDLE hFind = ::FindFirstFile(szFind, &FindFileData);

    if (INVALID_HANDLE_VALUE == hFind)
        return;

    while (TRUE)
    {
        if (FindFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
        {}
        else
        {
            if (strstr(FindFileData.cFileName, filePrefix) && !strstr(FindFileData.cFileName, "disp"))
            {
                char *szFile;
                szFile = (char*)malloc(TQC_MAX_PATH);

                memset(szFile, 0, TQC_MAX_PATH);
                strcpy(szFile, lpPath);
                strcat(szFile, "");
                strcat(szFile, FindFileData.cFileName);
                fileList.push_back(szFile);
            }
        }

        if (!FindNextFile(hFind, &FindFileData))
            break;
    }

    FindClose(hFind);
}

void AddFileList(const char *filePrefix, vector<char*> &fileList)
{
    char   path[TQC_MAX_PATH];
    char   filePre[TQC_MAX_PATH];
    size_t len    = strlen(filePrefix) - 1;
    size_t orgLen = len;

    memset(path, 0, TQC_MAX_PATH);
    memset(filePre, 0, TQC_MAX_PATH);

    while (filePrefix[len] != '\\')
    {
        len--;
    }

    memcpy(path, filePrefix, len + 1);
    memcpy(filePre, &filePrefix[len + 1], orgLen - len);

    AddFileList(path, filePre, fileList);
}

// Mouse event handler. Called automatically by OpenCV when the user clicks in the GUI window.
void OnMouse(int event, int x, int y, int, void*)
{
    // We only care about left-mouse clicks, not right-mouse clicks or mouse movement.
    if (event != CV_EVENT_LBUTTONDOWN)
        return;

    double q[4][4];
    Mat    _Q(4, 4, CV_64F, q);

    g_Q.convertTo(_Q, CV_64F);

    // Benet: we should multiply 16, because disparity value is multiplied by 16. See bm->compute.
    double xc = ((x + q[0][3]) / (q[3][2] * g_disp.at<short>(y, x) + q[3][3])) * 16;
    double yc = ((y + q[1][3]) / (q[3][2] * g_disp.at<short>(y, x) + q[3][3])) * 16;
    double zc = ((q[2][3]) / (q[3][2] * g_disp.at<short>(y, x) + q[3][3])) * 16;

    LOGE("(%d, %d, %d): %f, %f, %f\n", x, y, g_disp.at<short>(y, x), xc, yc, zc);
}



int main(int argc, char **argv)
{
    const char    *leftFilename         = 0;
    const char    *rightFilename        = 0;
    const char    *intrinsic_filename   = 0;
    const char    *extrinsic_filename   = 0;
    const char    *disparity_filename   = 0;
    const char    *point_cloud_filename = 0;
    const char    *leftPrefix           = NULL;
    const char    *rightPrefix          = NULL;
    vector<char*> fileList1;
    vector<char*> fileList2;

    int   alg                 = STEREO_SGBM;
    int   SADWindowSize       = 0;
    int   numberOfDisparities = 0;
    bool  no_display          = false;
    float scale               = 1.f;

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
            if (!leftFilename)
                leftFilename = argv[i];
            else
                rightFilename = argv[i];
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
            no_display = true;
        else if (strcmp(argv[i], "-i") == 0)
            intrinsic_filename = argv[++i];
        else if (strcmp(argv[i], "-e") == 0)
            extrinsic_filename = argv[++i];
        else if (strcmp(argv[i], "-o") == 0)
            disparity_filename = argv[++i];
        else if (strcmp(argv[i], "-p") == 0)
            point_cloud_filename = argv[++i];
        else if (strcmp(argv[i], "--left") == 0)
        {
            leftPrefix = argv[++i];
        }
        else if (strcmp(argv[i], "--right") == 0)
        {
            rightPrefix = argv[++i];
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

    if ((!leftFilename || !rightFilename) &&
        (!leftPrefix || !rightPrefix))
    {
        LOGE("Command-line parameter error: both left and right images must be specified\n");
        return -1;
    }

    if ((intrinsic_filename != 0) ^ (extrinsic_filename != 0))
    {
        LOGE("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
        return -1;
    }

    if (extrinsic_filename == 0 && point_cloud_filename && g_outputPath)
    {
        LOGE("Command-line parameter error: extrinsic and intrinsic parameters must be specified to compute the point cloud\n");
        return -1;
    }

    if (leftFilename && rightFilename)
    {
        fileList1.push_back((char*)leftFilename);
        fileList2.push_back((char*)rightFilename);
    }
    else if (leftPrefix && rightPrefix)
    {
        AddFileList(leftPrefix, fileList1);
        AddFileList(rightPrefix, fileList2);
    }

    for (int i = 0; i < fileList1.size() && i < fileList2.size(); i++)
    {
        int color_mode = alg == STEREO_BM ? 0 : -1;
        Mat img1       = imread(fileList1.at(i), color_mode);
        Mat img2       = imread(fileList2.at(i), color_mode);

        char   path[TQC_MAX_PATH];
        char   filePre[TQC_MAX_PATH];
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
            int k = 1;
            Mat temp1 = img1;
            Mat temp2 = img2;
            int width = g_width;
            int height = g_height;
            char buf[TQC_MAX_PATH];

            char   filePreRight[TQC_MAX_PATH];
            char   *rightFilePre = fileList2.at(i);
            size_t len = strlen(rightFilePre) - 1;
            size_t orgLen = len;

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

                width /= 2;
                height /= 2;

                resize(temp1, tmpScale1, Size(), 0.5f, 0.5f, INTER_AREA);
                resize(temp2, tmpScale2, Size(), 0.5f, 0.5f, INTER_AREA);

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

        if (intrinsic_filename)
        {
            // reading intrinsic parameters
            FileStorage fs(intrinsic_filename, FileStorage::READ);
            if (!fs.isOpened())
            {
                LOGE("Failed to open file %s\n", intrinsic_filename);
                return -1;
            }

            Mat M1, D1, M2, D2;
            fs["M1"] >> M1;
            fs["D1"] >> D1;
            fs["M2"] >> M2;
            fs["D2"] >> D2;

            M1 *= scale;
            M2 *= scale;

            fs.open(extrinsic_filename, FileStorage::READ);
            if (!fs.isOpened())
            {
                LOGE("Failed to open file %s\n", extrinsic_filename);
                return -1;
            }

            Mat R, T, R1, P1, R2, P2;
            fs["R"] >> R;
            fs["T"] >> T;

            if (R.cols == 1 && R.rows == 3)
            {
                Rodrigues(R, R);
            }

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

        // Output time cost.
        t = getTickCount() - t;
        SaveTimeCost(filePre, t);

        g_disp = disp;

        // disp = dispp.colRange(numberOfDisparities, img1p.cols);
        if (alg != STEREO_VAR)
            disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities * 16.));
        else
            disp.convertTo(disp8, CV_8U);

        // benet-add for matlab display
        if (disparity_filename)
        {
            // LOGE("Q Matrix: 0x%X\n", Q.type());
            // LOGE("%f %f %f %f\n", Q.at<double>(0, 0), Q.at<double>(0, 1), Q.at<double>(0, 2), Q.at<double>(0, 3));
            // LOGE("%f %f %f %f\n", Q.at<double>(1, 0), Q.at<double>(1, 1), Q.at<double>(1, 2), Q.at<double>(1, 3));
            // LOGE("%f %f %f %f\n", Q.at<double>(2, 0), Q.at<double>(2, 1), Q.at<double>(2, 2), Q.at<double>(2, 3));
            // LOGE("%f %f %f %f\n", Q.at<double>(3, 0), Q.at<double>(3, 1), Q.at<double>(3, 2), Q.at<double>(3, 3));
            SaveDispData(disparity_filename, filePre, disp);
        }

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

        if (point_cloud_filename)
        {
            LOGE("storing the point cloud...");
            fflush(stdout);
            Mat xyz;
            reprojectImageTo3D(disp, xyz, Q, true);
            SaveXYZData(point_cloud_filename, filePre, xyz);
            LOGE("\n");
        }

        SavePic(filePre, disp8);
    }

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
    CvMat grayMat = disp8;
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
    fp = fopen(buf, "wt");

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
#if SAVE_XYZ_FILTER
    const double max_z = 1.0e4;
#endif

    char *buf;
    FILE *fp = NULL;

    buf = GetFileName(filename, postfixName, "dat");
    fp = fopen(buf, "wt");

    for (int y = 0; y < mat.rows; y++)
    {
        for (int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);

#if SAVE_XYZ_FILTER
            if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z)
                continue;
#endif

            // fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
            fprintf(fp, "%f %f %f\n", point[0]*16, point[1]*16, point[2]*16);
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
    char fileName[TQC_MAX_PATH];
    char *strPicName = NULL;
    FILE *fp = NULL;
    float fTime = 0.0f;

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