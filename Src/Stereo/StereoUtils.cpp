#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include "TqcLog.h"
#include "TqcUtils.h"
#include "StereoUtils.h"

stCmdOption g_option;

bool ParseCmd(int argc, char *argv[], stCmdOption &cmd)
{
    if (argc < 3)
    {
        return false;
    }

    for (int i = 1; i < argc; i++)
    {
        if (strncmp(argv[i], TQC_ALGORITHM_OPTION, strlen(TQC_ALGORITHM_OPTION)) == 0)
        {
            cmd.strAlgorithmName = argv[i] + strlen(TQC_ALGORITHM_OPTION);
            cmd.algorithm        = strcmp(cmd.strAlgorithmName, TQC_ALGORITHM_NAME_BM) == 0 ? TQC_STEREO_BM :
                                   strcmp(cmd.strAlgorithmName, TQC_ALGORITHM_NAME_SGBM) == 0 ? TQC_STEREO_SGBM :
                                   strcmp(cmd.strAlgorithmName, TQC_ALGORITHM_NAME_HH) == 0 ? TQC_STEREO_HH :
                                   strcmp(cmd.strAlgorithmName, TQC_ALGORITHM_NAME_VAR) == 0 ? TQC_STEREO_VAR : TQC_STEREO_VALID;
            if (cmd.algorithm < 0)
            {
                LOGE("Command-line parameter error: Unknown stereo algorithm\n\n");
                return false;
            }
        }
        else if (strncmp(argv[i], TQC_MAX_DISPARITY_OPTION, strlen(TQC_MAX_DISPARITY_OPTION)) == 0)
        {
            if (sscanf(argv[i] + strlen(TQC_MAX_DISPARITY_OPTION), "%d", &cmd.nNumDisparities) != 1 ||
                cmd.nNumDisparities < 1 || cmd.nNumDisparities % 16 != 0)
            {
                LOGE("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
                return false;
            }
        }
        else if (strncmp(argv[i], TQC_BLOCK_SIZE_OPTION, strlen(TQC_BLOCK_SIZE_OPTION)) == 0)
        {
            if (sscanf(argv[i] + strlen(TQC_BLOCK_SIZE_OPTION), "%d", &cmd.nSADWindowSize) != 1 ||
                cmd.nSADWindowSize < 1 || cmd.nSADWindowSize % 2 != 1)
            {
                LOGE("Command-line parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
                return false;
            }
        }
        else if (strncmp(argv[i], TQC_SCALE_OPTION, strlen(TQC_SCALE_OPTION)) == 0)
        {
            if (sscanf(argv[i] + strlen(TQC_SCALE_OPTION), "%f", &cmd.fScale) != 1 || cmd.fScale < 0)
            {
                LOGE("Command-line parameter error: The scale factor (--scale=<...>) must be a positive floating-point number\n");
                return false;
            }
        }
        else if (strcmp(argv[i], TQC_NO_DISPLAY_OPTION) == 0)
        {
            cmd.bDisplay = false;
        }
        else if (strcmp(argv[i], "-i") == 0)
        {
            cmd.strIntrinsicFile = argv[++i];
        }
        else if (strcmp(argv[i], "-e") == 0)
        {
            cmd.strExtrinsicFile = argv[++i];
        }
        else if (strcmp(argv[i], "-o") == 0)
        {
            cmd.strDispFile = argv[++i];
        }
        else if (strcmp(argv[i], "-p") == 0)
        {
            cmd.strPCLFile = argv[++i];
        }
        else if (strcmp(argv[i], "-v") == 0)
        {
            cmd.strDepthFile = argv[++i];
            if (cmd.strDepthFile)
            {
                char buf[TQC_MAX_PATH];

                memset(buf, 0, TQC_MAX_PATH);
                sprintf(buf, "%s/%s", cmd.strOutputPath, cmd.strDepthFile);

                cmd.depthFile = fopen(buf, "w");
                if (!cmd.depthFile)
                {
                    LOGE("Cannot open depth file(%s).", buf);
                    return false;
                }
            }
        }
        else if (strcmp(argv[i], "--left") == 0)
        {
            cmd.strLeftPrefix = argv[++i];
        }
        else if (strcmp(argv[i], "--right") == 0)
        {
            cmd.strRightPrefix = argv[++i];
        }
        else if (strcmp(argv[i], "--path") == 0)
        {
            cmd.strOutputPath = argv[++i];
        }
        else
        {
            LOGE("Command-line parameter error: unknown option %s\n", argv[i]);
            return false;
        }
    }

    return true;
}

void PrintHelp()
{
    LOGE("\nDemo stereo matching converting L and R images into disparity and point clouds\n");
    LOGE("\nUsage: stereo_match <left_image> <right_image> [--algorithm=bm|sgbm|hh] [--blocksize=<block_size>]\n"
         "[--max-disparity=<max_disparity>] [--scale=scale_factor>] [-i <intrinsic_filename>] [-e <extrinsic_filename>]\n"
         "[--no-display] [-o <disparity_image>] [-p <point_cloud_file>]\n"
         "[--path outputPath] [--left left] [--right right]");
}

bool CheckOption(stCmdOption option)
{
    if ((!option.strLeftFile || !option.strRightFile) &&
        (!option.strLeftPrefix || !option.strRightPrefix))
    {
        LOGE("Command-line parameter error: both left and right images must be specified\n");
        return false;
    }

    if ((option.strIntrinsicFile != 0) ^ (option.strExtrinsicFile != 0))
    {
        LOGE("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
        return false;
    }

    if (option.strExtrinsicFile == 0 && option.strPCLFile && option.strOutputPath)
    {
        LOGE("Command-line parameter error: extrinsic and intrinsic parameters must be specified to compute the point cloud\n");
        return false;
    }

    return true;
}

bool GenerateMipmap(Mat img1, Mat img2, int width, int height, const char *filePreLeft, const char *filePreRight)
{
    int  k     = 1;
    Mat  temp1 = img1;
    Mat  temp2 = img2;
    char buf[TQC_MAX_PATH];

    do
    {
        Mat tmpScale1;
        Mat tmpScale2;

        width  /= 2;
        height /= 2;

        resize(temp1, tmpScale1, Size(), 0.5f, 0.5f, INTER_LINEAR);
        resize(temp2, tmpScale2, Size(), 0.5f, 0.5f, INTER_LINEAR);

        memset(buf, 0, 256);
        sprintf(buf, "%s/%s_%d.jpg", g_option.strOutputPath, filePreLeft, 20 + k);
        imwrite(buf, temp1);

        memset(buf, 0, 256);
        sprintf(buf, "%s/%s_%d.jpg", g_option.strOutputPath, filePreRight, 20 + k);
        imwrite(buf, temp2);

        k++;

        temp1 = tmpScale1;
        temp2 = tmpScale2;
    }
    while (width > 32 && height > 32);

    return true;
}

void Gray2Color(CvMat *pGrayMat, CvMat *pColorMat)
{
    if (pColorMat)
        cvZero(pColorMat);

    int stype = CV_MAT_TYPE(pGrayMat->type), dtype = CV_MAT_TYPE(pColorMat->type);
    int rows  = pGrayMat->rows;
    int cols  = pGrayMat->cols;

    // 1. Check size of color and gray image.
    // 2. Check images' type.
    if (CV_ARE_SIZES_EQ(pGrayMat, pColorMat) && stype == CV_8UC1 && dtype == CV_8UC3)
    {
        CvMat *red   = cvCreateMat(rows, cols, CV_8U);
        CvMat *green = cvCreateMat(rows, cols, CV_8U);
        CvMat *blue  = cvCreateMat(rows, cols, CV_8U);
        CvMat *mask  = cvCreateMat(rows, cols, CV_8U);

        // Calculate each channel's value of color image.
        cvSubRS(pGrayMat, cvScalar(255), blue); // blue(I) = 255 - gray(I)
        cvCopy(pGrayMat, red);                  // red(I) = gray(I)
        cvCopy(pGrayMat, green);                // green(I) = gray(I),if gray(I) < 128
        cvCmpS(green, 128, mask, CV_CMP_GE);    // green(I) = 255 - gray(I), if gray(I) >= 128
        cvSubRS(green, cvScalar(255), green, mask);
        cvConvertScale(green, green, 2.0, 0.0);

        // Merge R,G,B channel to one image.
        cvMerge(blue, green, red, NULL, pColorMat);

        cvReleaseMat(&red);
        cvReleaseMat(&green);
        cvReleaseMat(&blue);
        cvReleaseMat(&mask);
    }
}

char* GetFileName(const char *fileName,
                  const char *postfixName,
                  const char *extName,
                  const char *strOutputPath,
                  const char *strAlgorithmName,
                  int width,
                  int height)
{
    static char output[TQC_MAX_PATH];

    memset(output, 0, TQC_MAX_PATH);
    sprintf(output, "%s/%s_%s_%dx%d_%s.%s", strOutputPath, fileName, strAlgorithmName, width, height, postfixName, extName);

    return output;
}

void SavePic(const char *postfixName, const char *strOutputPath, const char *strAlgorithmName, Mat &disp8)
{
    char *strFileName;

    // Save disparity picture
    strFileName = GetFileName("disp", postfixName, "jpg", strOutputPath, strAlgorithmName, disp8.cols, disp8.rows);
    imwrite(strFileName, disp8);

    // Save color picture
    CvMat *pColorMat = cvCreateMat(disp8.rows, disp8.cols, CV_8UC3);
    CvMat grayMat    = disp8;
    Gray2Color(&grayMat, pColorMat);
    strFileName = GetFileName("color", postfixName, "jpg", strOutputPath, strAlgorithmName, disp8.cols, disp8.rows);
    Mat colorMat1 = Mat(pColorMat->rows, pColorMat->cols, CV_8UC3, pColorMat->data.ptr);
    imwrite(strFileName, colorMat1);
    cvReleaseMat(&pColorMat);
}

void SaveDispData(const char *filename, const char *postfixName, const char *strOutputPath, const char *strAlgorithmName, const Mat &mat)
{
    char *buf;
    FILE *fp = NULL;

    buf = GetFileName(filename, postfixName, "dat", strOutputPath, strAlgorithmName, mat.cols, mat.rows);
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

void SaveXYZData(const char *filename, const char *postfixName, const char *strOutputPath, const char *strAlgorithmName, const Mat &mat)
{
#if TQC_SAVE_XYZ_FILTER
    const double max_z = 1.0e4;
#endif

    char *buf;
    FILE *fp = NULL;

    buf = GetFileName(filename, postfixName, "dat", strOutputPath, strAlgorithmName, mat.cols, mat.rows);
    fp = fopen(buf, "wt");

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

void StereoReprojectPixelTo3D(const Mat disp, const Mat &Q, const Point2i &pixel, Point3d &point)
{
    double q[4][4];
    Mat    _Q(4, 4, CV_64F, q);
    int    x = pixel.x;
    int    y = pixel.y;

    Q.convertTo(_Q, CV_64F);

    point.x = ((x + q[0][3]) / (q[3][2] * disp.at<short>(y, x) + q[3][3])) * 16;
    point.y = ((y + q[1][3]) / (q[3][2] * disp.at<short>(y, x) + q[3][3])) * 16;
    point.z = ((q[2][3]) / (q[3][2] * disp.at<short>(y, x) + q[3][3])) * 16;
}