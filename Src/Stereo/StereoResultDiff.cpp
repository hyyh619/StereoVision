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

char *g_leftPrefix;
char *g_rightPrefix;
char *g_sourcePic;

int main(int argc, char **argv)
{
    vector<char*> fileList1;
    vector<char*> fileList2;
    vector<char*> srcFileList;

    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--prefix") == 0)
        {
            g_leftPrefix  = argv[++i];
            g_rightPrefix = argv[++i];
        }
        else if (strcmp(argv[i], "--source") == 0)
        {
            g_sourcePic = argv[++i];
        }
    }

    if (g_leftPrefix && g_rightPrefix)
    {
        AddFileList(g_leftPrefix, fileList1);
        AddFileList(g_rightPrefix, fileList2);
        AddFileList(g_sourcePic, srcFileList);
    }
    else
    {
        LOGE("Cannot get prefix.");
        return -1;
    }

    if (fileList1.size() != fileList2.size())
    {
        LOGE("fileList1 size(%d) != fileList2 size(%d)", fileList1.size(), fileList2.size());
        return -1;
    }

    namedWindow("source", 1);
    namedWindow("left", 1);
    namedWindow("right", 1);

    for (int i = 0; i < fileList1.size() && i < fileList2.size(); i++)
    {
        Mat img1 = imread(fileList1.at(i));
        Mat img2 = imread(fileList2.at(i));
        Mat src  = imread(srcFileList.at(i));

        imshow("source", src);
        imshow("left", img1);
        imshow("right", img2);

        char key = waitKey();
        if (key == VK_ESCAPE)
        {
            break;
        }
    }
}