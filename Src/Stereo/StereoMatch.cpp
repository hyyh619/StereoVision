#include <stdio.h>
#include <string.h>
#include <vector>
#include <windows.h>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"

using namespace cv;
using namespace std;

#define MAX_PATH 256

enum
{
    STEREO_BM   = 0,
    STEREO_SGBM = 1,
    STEREO_HH   = 2,
    STEREO_VAR  = 3
};

Mat g_disp;
Mat g_Q;

void SaveDisp(const char *filename, const Mat &mat);
void Gray2Color(CvMat *pGrayMat, CvMat *pColorMat);

static void PrintHelp()
{
    printf("\nDemo stereo matching converting L and R images into disparity and point clouds\n");
    printf("\nUsage: stereo_match <left_image> <right_image> [--algorithm=bm|sgbm|hh] [--blocksize=<block_size>]\n"
           "[--max-disparity=<max_disparity>] [--scale=scale_factor>] [-i <intrinsic_filename>] [-e <extrinsic_filename>]\n"
           "[--no-display] [-o <disparity_image>] [-p <point_cloud_file>]\n"
           "[--path outputPath] [--left left] [--right right]");
}

static void SaveXYZ(const char *filename, const Mat &mat)
{
    const double max_z = 1.0e4;
    FILE         *fp   = fopen(filename, "wt");

    for (int y = 0; y < mat.rows; y++)
    {
        for (int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            // if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z)
            //    continue;
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
        }
    }

    fclose(fp);
}

void AddFileList(const char *lpPath, const char *filePrefix, vector<char*> &fileList)
{
    char            szFind[MAX_PATH];
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
                szFile = (char*)malloc(MAX_PATH);

                memset(szFile, 0, MAX_PATH);
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
    char path[MAX_PATH];
    char filePre[MAX_PATH];
    int  len    = strlen(filePrefix) - 1;
    int  orgLen = len;

    memset(path, 0, MAX_PATH);
    memset(filePre, 0, MAX_PATH);

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

    printf("(%d, %d, %d): %f, %f, %f\n", x, y, g_disp.at<short>(y, x), xc, yc, zc);
}


int main(int argc, char **argv)
{
    char       strFileName[MAX_PATH];
    const char *algorithm_opt = "--algorithm=";
    const char *maxdisp_opt   = "--max-disparity=";
    const char *blocksize_opt = "--blocksize=";
    const char *nodisplay_opt = "--no-display";
    const char *scale_opt     = "--scale=";

    if (argc < 3)
    {
        PrintHelp();
        return 0;
    }

    const char    *leftFilename         = 0;
    const char    *rightFilename        = 0;
    const char    *intrinsic_filename   = 0;
    const char    *extrinsic_filename   = 0;
    const char    *disparity_filename   = 0;
    const char    *point_cloud_filename = 0;
    const char    *leftPrefix           = NULL;
    const char    *rightPrefix          = NULL;
    const char    *outputPath           = NULL;
    vector<char*> fileList1;
    vector<char*> fileList2;

    int   alg                 = STEREO_SGBM;
    int   SADWindowSize       = 0;
    int   numberOfDisparities = 0;
    bool  no_display          = false;
    float scale               = 1.f;
    char  *algorithmName;

    Ptr<StereoBM>   bm   = StereoBM::create(16, 9);
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);

    for (int i = 1; i < argc; i++)
    {
        if (argv[i][0] != '-')
        {
            if (!leftFilename)
                leftFilename = argv[i];
            else
                rightFilename = argv[i];
        }
        else if (strncmp(argv[i], algorithm_opt, strlen(algorithm_opt)) == 0)
        {
            algorithmName = argv[i] + strlen(algorithm_opt);
            alg           = strcmp(algorithmName, "bm") == 0 ? STEREO_BM :
                            strcmp(algorithmName, "sgbm") == 0 ? STEREO_SGBM :
                            strcmp(algorithmName, "hh") == 0 ? STEREO_HH :
                            strcmp(algorithmName, "var") == 0 ? STEREO_VAR : -1;
            if (alg < 0)
            {
                printf("Command-line parameter error: Unknown stereo algorithm\n\n");
                PrintHelp();
                return -1;
            }
        }
        else if (strncmp(argv[i], maxdisp_opt, strlen(maxdisp_opt)) == 0)
        {
            if (sscanf(argv[i] + strlen(maxdisp_opt), "%d", &numberOfDisparities) != 1 ||
                numberOfDisparities < 1 || numberOfDisparities % 16 != 0)
            {
                printf("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
                PrintHelp();
                return -1;
            }
        }
        else if (strncmp(argv[i], blocksize_opt, strlen(blocksize_opt)) == 0)
        {
            if (sscanf(argv[i] + strlen(blocksize_opt), "%d", &SADWindowSize) != 1 ||
                SADWindowSize < 1 || SADWindowSize % 2 != 1)
            {
                printf("Command-line parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
                return -1;
            }
        }
        else if (strncmp(argv[i], scale_opt, strlen(scale_opt)) == 0)
        {
            if (sscanf(argv[i] + strlen(scale_opt), "%f", &scale) != 1 || scale < 0)
            {
                printf("Command-line parameter error: The scale factor (--scale=<...>) must be a positive floating-point number\n");
                return -1;
            }
        }
        else if (strcmp(argv[i], nodisplay_opt) == 0)
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
            outputPath = argv[++i];
        }
        else
        {
            printf("Command-line parameter error: unknown option %s\n", argv[i]);
            return -1;
        }
    }

    if ((!leftFilename || !rightFilename) &&
        (!leftPrefix || !rightPrefix))
    {
        printf("Command-line parameter error: both left and right images must be specified\n");
        return -1;
    }

    if ((intrinsic_filename != 0) ^ (extrinsic_filename != 0))
    {
        printf("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
        return -1;
    }

    if (extrinsic_filename == 0 && point_cloud_filename && outputPath)
    {
        printf("Command-line parameter error: extrinsic and intrinsic parameters must be specified to compute the point cloud\n");
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

        if (img1.empty())
        {
            printf("Command-line parameter error: could not load the first input image file\n");
            return -1;
        }

        if (img2.empty())
        {
            printf("Command-line parameter error: could not load the second input image file\n");
            return -1;
        }

        if (scale != 1.f)
        {
            Mat temp1, temp2;
            int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
            resize(img1, temp1, Size(), scale, scale, method);
            img1 = temp1;
            resize(img2, temp2, Size(), scale, scale, method);
            img2 = temp2;
        }

        Size img_size = img1.size();

        Rect roi1, roi2;
        Mat  Q;

        if (intrinsic_filename)
        {
            // reading intrinsic parameters
            FileStorage fs(intrinsic_filename, FileStorage::READ);
            if (!fs.isOpened())
            {
                printf("Failed to open file %s\n", intrinsic_filename);
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
                printf("Failed to open file %s\n", extrinsic_filename);
                return -1;
            }

            Mat R, T, R1, P1, R2, P2;
            fs["R"] >> R;
            fs["T"] >> T;

            if (R.cols == 1 && R.rows == 3)
            {
                Rodrigues(R, R);
            }

            stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);

            Mat map11, map12, map21, map22;
            initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
            initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

            Mat img1r, img2r;
            remap(img1, img1r, map11, map12, INTER_LINEAR);
            remap(img2, img2r, map21, map22, INTER_LINEAR);

            img1 = img1r;
            img2 = img2r;

            g_Q = Q;
        }

        numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width / 8) + 15) & - 16;

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
        // Mat img1p, img2p, dispp;
        // copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
        // copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

        int64 t = getTickCount();

        if (alg == STEREO_BM)
        {
            bm->compute(img1, img2, disp);
        }
        else if (alg == STEREO_SGBM || alg == STEREO_HH)
        {
            sgbm->compute(img1, img2, disp);
        }

        g_disp = disp;

        t = getTickCount() - t;
        printf("#%d---Time elapsed: %fms\n", i, t * 1000 / getTickFrequency());

        // disp = dispp.colRange(numberOfDisparities, img1p.cols);
        if (alg != STEREO_VAR)
            disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities * 16.));
        else
            disp.convertTo(disp8, CV_8U);

        // benet-add for matlab display
        // if (disparity_filename)
        // {
        //    printf("Q Matrix: 0x%X\n", Q.type());
        //    printf("%f %f %f %f\n", Q.at<double>(0, 0), Q.at<double>(0, 1), Q.at<double>(0, 2), Q.at<double>(0, 3));
        //    printf("%f %f %f %f\n", Q.at<double>(1, 0), Q.at<double>(1, 1), Q.at<double>(1, 2), Q.at<double>(1, 3));
        //    printf("%f %f %f %f\n", Q.at<double>(2, 0), Q.at<double>(2, 1), Q.at<double>(2, 2), Q.at<double>(2, 3));
        //    printf("%f %f %f %f\n", Q.at<double>(3, 0), Q.at<double>(3, 1), Q.at<double>(3, 2), Q.at<double>(3, 3));
        //    SaveDisp(disparity_filename, disp);
        // }

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

            printf("press any key to continue...");
            fflush(stdout);
            waitKey();
            printf("\n");
        }

        memset(strFileName, 0, MAX_PATH);
        sprintf(strFileName, "%s_disp_%s.jpg", fileList1.at(i), algorithmName);
        imwrite(strFileName, disp8);

        // if (point_cloud_filename)
        // {
        //    printf("storing the point cloud...");
        //    fflush(stdout);
        //    Mat xyz;
        //    reprojectImageTo3D(disp, xyz, Q, true);
        //    SaveXYZ(point_cloud_filename, xyz);
        //    printf("\n");
        // }

        CvMat *pColorMat = cvCreateMat(disp8.rows, disp8.cols, CV_8UC3);
        CvMat grayMat    = disp8;
        Gray2Color(&grayMat, pColorMat);
        memset(strFileName, 0, MAX_PATH);
        sprintf(strFileName, "%s_color_%s.jpg", fileList1.at(i), algorithmName);
        Mat colorMat1 = Mat(pColorMat->rows, pColorMat->cols, CV_8UC3, pColorMat->data.ptr);
        imwrite(strFileName, colorMat1);
        cvReleaseMat(&pColorMat);
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

void SaveDisp(const char *filename, const Mat &mat)
{
    FILE *fp = fopen(filename, "wt");

    fprintf(fp, "%02d\n", mat.rows);
    fprintf(fp, "%02d\n", mat.cols);

    for (int y = 0; y < mat.rows; y++)
    {
        for (int x = 0; x < mat.cols; x++)
        {
            int disp = (int)mat.at<short>(y, x);    // 这里视差矩阵是CV_16S 格式的，故用 short 类型读取
            fprintf(fp, "%d %d %d\n", x, y, disp);              // 若视差矩阵是 CV_32F 格式，则用 float 类型读取
        }
    }

    fclose(fp);
}

void Gray2Color(CvMat *pGrayMat, CvMat *pColorMat)
{
    if (pColorMat)
        cvZero(pColorMat);

    int stype = CV_MAT_TYPE(pGrayMat->type), dtype = CV_MAT_TYPE(pColorMat->type);
    int rows  = pGrayMat->rows, cols = pGrayMat->cols;

    // 判断输入的灰度图和输出的伪彩色图是否大小相同、格式是否符合要求
    if (CV_ARE_SIZES_EQ(pGrayMat, pColorMat) && stype == CV_8UC1 && dtype == CV_8UC3)
    {
        CvMat *red   = cvCreateMat(pGrayMat->rows, pGrayMat->cols, CV_8U);
        CvMat *green = cvCreateMat(pGrayMat->rows, pGrayMat->cols, CV_8U);
        CvMat *blue  = cvCreateMat(pGrayMat->rows, pGrayMat->cols, CV_8U);
        CvMat *mask  = cvCreateMat(pGrayMat->rows, pGrayMat->cols, CV_8U);

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