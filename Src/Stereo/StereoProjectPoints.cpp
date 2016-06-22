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

static bool ReadStringList(const string &filename, vector<string> &l)
{
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;

    FileNode n = fs.getFirstTopLevelNode();
    if (n.type() != FileNode::SEQ)
        return false;

    FileNodeIterator it = n.begin(), it_end = n.end();

    for (; it != it_end; ++it)
        l.push_back((string) * it);

    return true;
}

static bool ReadCameraMatrix(const string &filename,
                             Mat &cameraMatrix,
                             Mat &distCoeffs,
                             Size &calibratedImageSize)
{
    // reading intrinsic parameters
    FileStorage fs(filename, FileStorage::READ);

    if (!fs.isOpened())
    {
        LOGE("Failed to open file %s\n", filename);
        return false;
    }

    Mat M1, D1, M2, D2;
    fs["M1"] >> cameraMatrix;
    fs["D1"] >> distCoeffs;

    calibratedImageSize.width  = 640;
    calibratedImageSize.height = 480;

    return true;
}

static void CalcChessboardCorners(Size boardSize, float squareSize, vector<Point3f> &corners)
{
    corners.resize(0);

    for (int i = 0; i < boardSize.height; i++)
    {
        for (int j = 0; j < boardSize.width; j++)
        {
            corners.push_back(Point3f(float(j * squareSize), float(i * squareSize), 0));
        }
    }
}

int main(int argc, char **argv)
{
    const char   *intrinsicsFilename = NULL;
    const char   *inputName          = NULL;
    Mat          cameraMat;
    Mat          distortMat;
    Mat          distortMatOrig;
    Size         imgSize;
    int          cameraId;
    VideoCapture capture;

    vector<string>  imageList;
    Mat             frame;
    Mat             mapxy;
    bool            boardFound = false;
    Size            boardSize;
    vector<Point3f> box;
    vector<Point3f> boardPoints;
    Mat             rvec = Mat(3, 1, CV_64F);
    Mat             tvec = Mat(3, 1, CV_64F);
    double          squareSize = 1;
    Mat             shownFrame;
    bool            bFoundCorner = false;

    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "-i") == 0)
        {
            intrinsicsFilename = argv[++i];
        }
        else if (argv[i][0] != '-')
        {
            if (isdigit(argv[i][0]))
                sscanf_s(argv[i], "%d", &cameraId);
            else
                inputName = argv[i];
        }
        else if (strcmp(argv[i], "-w") == 0)
        {
            if (sscanf_s(argv[++i], "%d", &boardSize.width) != 1 || boardSize.width <= 0)
            {
                LOGE("Incorrect -w parameter (must be a positive integer)\n");
                return 0;
            }
        }
        else if (strcmp(argv[i], "-h") == 0)
        {
            if (sscanf_s(argv[++i], "%d", &boardSize.height) != 1 || boardSize.height <= 0)
            {
                LOGE("Incorrect -h parameter (must be a positive integer)\n");
                return 0;
            }
        }
        else if (strcmp(argv[i], "-s") == 0)
        {
            if (sscanf_s(argv[++i], "%lf", &squareSize) != 1 || squareSize <= 0)
            {
                LOGE("Incorrect -w parameter (must be a positive real number)\n");
                return 0;
            }
        }
        else
        {
            LOGE("Incorrect option\n");
            return 0;
        }
    }

    ReadCameraMatrix(intrinsicsFilename, cameraMat, distortMat, imgSize);
    distortMat.copyTo(distortMatOrig);

    if (inputName)
    {
        if (!ReadStringList(inputName, imageList) &&
            !capture.open(inputName))
        {
            LOGE("The input file could not be opened\n");
            return -1;
        }
    }
    else
    {
        capture.open(cameraId);

        // Try to set the camera resolution. Note that this only works for some cameras on
        // some computers and only for some drivers, so don't rely on it to work!
        capture.set(CV_CAP_PROP_FRAME_WIDTH, 320);
        capture.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
    }

    if (!capture.isOpened() && imageList.empty())
    {
        LOGE("Could not initialize video capture\n");
        return -2;
    }

    namedWindow("view", 1);
    CalcChessboardCorners(boardSize, (float)squareSize, boardPoints);

    for (int i = 0;; i++)
    {
        Mat frame0;
        vector<Point2f> foundBoardCorners;

        if (!imageList.empty())
        {
            if (i < (int)imageList.size())
            {
                frame0 = imread(string(imageList[i]), 1);
            }
        }
        else
        {
            capture >> frame0;
        }

        if (frame0.empty())
            break;

        if (frame.empty())
        {
            if (frame0.size() != imgSize)
            {
                double sx = (double)frame0.cols / imgSize.width;
                double sy = (double)frame0.rows / imgSize.height;

                // adjust the camera matrix for the new resolution
                cameraMat.at<double>(0, 0) *= sx;
                cameraMat.at<double>(0, 2) *= sx;
                cameraMat.at<double>(1, 1) *= sy;
                cameraMat.at<double>(1, 2) *= sy;
            }

            Mat dummy;
            initUndistortRectifyMap(cameraMat, distortMat, Mat(),
                                    cameraMat, frame0.size(),
                                    CV_32FC2, mapxy, dummy);
            distortMat = Mat::zeros(5, 1, CV_64F);
        }

        remap(frame0, frame, mapxy, Mat(), INTER_LINEAR);

        //if (1)
        //{
        //    namedWindow("src", 1);
        //    imshow("src", frame0);
        //    namedWindow("dst", 1);
        //    imshow("dst", frame);

        //    LOGE("press any key to continue...");
        //    fflush(stdout);
        //    waitKey();
        //}

        if (bFoundCorner)
        {
            boardFound = findChessboardCorners(frame, boardSize, foundBoardCorners);
            if (boardFound)
            {
                solvePnP(Mat(boardPoints), Mat(foundBoardCorners), cameraMat,
                    distortMat, rvec, tvec, false);

                LOGE("rvec(%f, %f, %f), tvec(%f, %f, %f)",
                    rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0),
                    tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(1, 0));

                frame.copyTo(shownFrame);
                drawChessboardCorners(shownFrame, boardSize, Mat(foundBoardCorners), boardFound);
                imshow("view", shownFrame);
                // waitKey();
            }
        }
        else
        {
            frame.copyTo(shownFrame);
            imshow("view", shownFrame);
        }


        char keypress = waitKey(20);  // This is needed if you want to see anything!
        if (keypress == VK_ESCAPE)
        {
            // Escape Key
            // Quit the program!
            break;
        }
        else if (keypress == VK_RETURN)
        {
            vector<Point3f> tempobj(4);
            vector<Point2f> imgpt(4);

            //tempobj[0] = Point3f(-500.0f, -500.0f, 1000.0f);
            //tempobj[1] = Point3f(500.0f, -500.0f, 1000.0f);
            //tempobj[2] = Point3f(500.0f, 500.0f, 1000.0f);
            //tempobj[3] = Point3f(-500.0f, 500.0f, 1000.0f);

            //// X, Y, Z: 40cm x 40cm x 1500cm
            //tempobj[0] = Point3f(-200.0f, -200.0f, 1500.0f);
            //tempobj[1] = Point3f(200.0f, -200.0f, 1500.0f);
            //tempobj[2] = Point3f(200.0f, 200.0f, 1500.0f);
            //tempobj[3] = Point3f(-200.0f, 200.0f, 1500.0f);

            // X, Y, Z: 30cm x 30cm x 1000cm
            tempobj[0] = Point3f(-150.0f, -150.0f, 1000.0f);
            tempobj[1] = Point3f(150.0f, -150.0f, 1000.0f);
            tempobj[2] = Point3f(150.0f, 150.0f, 1000.0f);
            tempobj[3] = Point3f(-150.0f, 150.0f, 1000.0f);

            rvec.at<double>(0, 0) = 0.0f;
            rvec.at<double>(1, 0) = 0.0f;
            rvec.at<double>(2, 0) = 0.0f;

            tvec.at<double>(0, 0) = 0.0f;
            tvec.at<double>(1, 0) = 0.0f;
            tvec.at<double>(2, 0) = 0.0f;

            projectPoints((Mat)tempobj, rvec, tvec, cameraMat, Mat(), imgpt);

            LOGE("image(%d, %d)", frame.cols, frame.rows);
            LOGE("%f, %f", imgpt[0].x, imgpt[0].y);
            LOGE("%f, %f", imgpt[1].x, imgpt[1].y);
            LOGE("%f, %f", imgpt[2].x, imgpt[2].y);
            LOGE("%f, %f", imgpt[3].x, imgpt[3].y);
            LOGE("wxh(%f, %f)", imgpt[2].x - imgpt[0].x, imgpt[2].y - imgpt[0].y);

            imwrite("projectPoints_320x240_1m.jpg", frame);
        }
    }
}