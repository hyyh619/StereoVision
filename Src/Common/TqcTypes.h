/**
@ TqcTypes.h
@ data types definition.
@ v1.0 2016.2.16 by Benet Huang
*/

#ifndef __TQC_TYPES_H
#define __TQC_TYPES_H

#include <opencv2/opencv.hpp>
#include "TqcConfig.h"
#include "TqcOptions.h"

using namespace cv;
using namespace cv::face;

#if TQC_USE_OPENCL
typedef UMat        MAT;
#else
typedef Mat         MAT;
#endif

#ifdef TQC_USE_LBPH_FACE
#define FaceRecognizerType          LBPHFaceRecognizer
#else
#define FaceRecognizerType          BasicFaceRecognizer
#endif

#endif /* __TQC_TYPES_H */