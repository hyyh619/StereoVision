#include <opencv2/imgproc/imgproc.hpp>

#include "TqcLog.h"
#include "Config.h"
#include "StereoMatchAlgorithm.h"
#include "StereoUtils.h"

stAlgorithmParam g_algorithmParam;
Ptr<StereoBM>    g_bm   = StereoBM::create(16, 9);
Ptr<StereoSGBM>  g_sgbm = StereoSGBM::create(0, 16, 3);

bool StereoInitAlgorithm(int nChannels,
                         Rect roi1,
                         Rect roi2,
                         int nNumDisparities,
                         int nSADWindowSize,
                         int imgWidth,
                         enAlgorithm selector)
{
    nNumDisparities = nNumDisparities > 0 ? nNumDisparities : ((imgWidth / 8) + 15) & - 16;

    switch (selector)
    {
    case TQC_STEREO_BM:
        nSADWindowSize = nSADWindowSize > 0 ? nSADWindowSize : 9;
        g_bm->setROI1(roi1);
        g_bm->setROI2(roi2);
        g_bm->setPreFilterCap(31);
        g_bm->setBlockSize(nSADWindowSize);
        g_bm->setMinDisparity(0);
        g_bm->setNumDisparities(nNumDisparities);
        g_bm->setTextureThreshold(10);
        g_bm->setUniquenessRatio(15);
        g_bm->setSpeckleWindowSize(100);
        g_bm->setSpeckleRange(32);
        g_bm->setDisp12MaxDiff(1);
        break;

    case TQC_STEREO_SGBM:
    case TQC_STEREO_HH:
        nSADWindowSize = nSADWindowSize > 0 ? nSADWindowSize : 3;
        g_sgbm->setPreFilterCap(63);
        g_sgbm->setBlockSize(nSADWindowSize);
        g_sgbm->setP1(8 * nChannels * nSADWindowSize * nSADWindowSize);
        g_sgbm->setP2(32 * nChannels * nSADWindowSize * nSADWindowSize);
        g_sgbm->setMinDisparity(0);
        g_sgbm->setNumDisparities(nNumDisparities);
        g_sgbm->setUniquenessRatio(10);
        g_sgbm->setSpeckleWindowSize(100);
        g_sgbm->setSpeckleRange(32);
        g_sgbm->setDisp12MaxDiff(1);
        g_sgbm->setMode(selector == TQC_STEREO_HH ? StereoSGBM::MODE_HH : StereoSGBM::MODE_SGBM);
        break;

    default:
        LOGE("%s(%d): wrong algorithm selector(%d)", __FUNCTION__, __LINE__, selector);
        return false;
    }

    g_algorithmParam.nNumDisparities = nNumDisparities;
    g_algorithmParam.nSADWindowSize  = nSADWindowSize;
    g_algorithmParam.nImgWidth       = imgWidth;
    g_algorithmParam.selector        = selector;

    return true;
}

bool StereoMatch(Mat left,
                 Mat right,
                 float fScale,
                 enAlgorithm selector,
                 stCamParam camParam,
                 Mat &disp)
{
    Mat imgLeft;
    Mat imgRight;
    Mat img1r;
    Mat img2r;

    if (fScale != 1.f)
    {
        Mat temp1, temp2;
        int method = fScale < 1 ? INTER_AREA : INTER_CUBIC;
        resize(left, temp1, Size(), fScale, fScale, method);
        imgLeft = temp1;
        resize(right, temp2, Size(), fScale, fScale, method);
        imgRight = temp2;
    }
    else
    {
        imgLeft  = left;
        imgRight = right;
    }

    remap(imgLeft, img1r, camParam.map11, camParam.map12, INTER_LINEAR);
    remap(imgRight, img2r, camParam.map21, camParam.map22, INTER_LINEAR);

    imgLeft  = img1r;
    imgRight = img2r;

#if TQC_STEREO_CULL
    {
        Rect dstRC;
        Mat  dstROI;

        // Get the destination ROI (and make sure it is within the image!).
        dstRC = Rect(TQC_STEREO_CAMERA_X_BORDER, TQC_STEREO_CAMERA_Y_BORDER,
                     imgLeft.cols - TQC_STEREO_CAMERA_X_BORDER * 2, imgRight.rows - TQC_STEREO_CAMERA_Y_BORDER * 2);
        imgLeft  = imgLeft(dstRC);
        imgRight = imgRight(dstRC);
    }
#endif

    if (selector == TQC_STEREO_BM)
    {
        g_bm->compute(imgLeft, imgRight, disp);
    }
    else if (selector == TQC_STEREO_SGBM || selector == TQC_STEREO_HH)
    {
        g_sgbm->compute(imgLeft, imgRight, disp);
    }

    return true;
}

Mat StereoGetDisp8FromDisp(Mat disp, enAlgorithm selector, int nNumDisparities)
{
    Mat disp8;

    if (selector != TQC_STEREO_VAR)
    {
        disp.convertTo(disp8, CV_8U, 255 / (nNumDisparities * 16.));
    }
    else
    {
        disp.convertTo(disp8, CV_8U);
    }

    return disp8;
}

double StereoGetDepthFromPixel(const Mat disp, const Mat &Q, const Point2i &pixel)
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

void StereoCalcDepthOfVirtualCopter(const Mat &disp, const Mat &Q, double d[3][3])
{
    for (int j = 0; j < TQC_VIRTUAL_COPTER_Y_SPLITE; j++)
    {
        for (int i = 0; i < TQC_VIRTUAL_COPTER_X_SPLITE; i++)
        {
            int    left = TQC_VIRTUAL_COPTER_LEFT + TQC_VIRTUAL_COPTER_SUB_X * i;
            int    top = TQC_VIRTUAL_COPTER_TOP + TQC_VIRTUAL_COPTER_SUB_Y * j;
            int    right = left + TQC_VIRTUAL_COPTER_SUB_X;
            int    bottom = top + TQC_VIRTUAL_COPTER_SUB_Y;
            double dMin = TQC_MAX_DEPTH;

            for (int y = top; y < bottom; y++)
            {
                for (int x = left; x < right; x++)
                {
                    double cur = StereoGetDepthFromPixel(disp, Q, Point2i(x, y));
                    if (dMin > cur && cur > FLT_EPSILON)
                        dMin = cur;
                }
            }

            d[j][i] = dMin;
        }
    }
}

void StereoFilterDisp(Mat &disp, Mat Q)
{
    double q[4][4];
    Mat    _Q(4, 4, CV_64F, q);

    Q.convertTo(_Q, CV_64F);

    for (int y = 0; y < disp.rows; y++)
    {
        for (int x = 0; x < disp.cols; x++)
        {
            int    disp8 = (int)disp.at<short>(y, x);
            double zc = ((q[2][3]) / (q[3][2] * disp8 + q[3][3])) * 16;

            // Filter, if depth > 5m, we will skip this.
            if (zc > 5000.0f)
            {
                disp.at<short>(y, x) = -16;
            }
        }
    }
}