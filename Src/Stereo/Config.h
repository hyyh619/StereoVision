#ifndef __CONFIG_H
#define __CONFIG_H

// Match Algorithm's number of disparities
#ifndef TQC_NUM_DISPARITIES
#define TQC_NUM_DISPARITIES 16
#endif

// SAD Window Size
#ifndef TQC_SAD_WINDOW_SIZE
#define TQC_SAD_WINDOW_SIZE 5
#endif

// Image scale, 1.0 means no scale.
#ifndef TQC_IMAGE_SCALE
#define TQC_IMAGE_SCALE 1.0f
#endif

// Cull source images from 320x240 to 300x230
#ifndef TQC_STEREO_CULL
#define TQC_STEREO_CULL 1
#endif

// Enable filter depth value if value > some value.
#ifndef TQC_FILTER_DEPTH_VALUE
#define TQC_FILTER_DEPTH_VALUE 0
#endif

// Output virtual copter's depth value to file
#ifndef TQC_OUTPUT_VIRTUAL_COPTER_DEPTH_TO_FILE
#define TQC_OUTPUT_VIRTUAL_COPTER_DEPTH_TO_FILE 1
#endif

// Dump disparity values to file.
#ifndef TQC_OUTPUT_DISP_VALUE_TO_FILE
#define TQC_OUTPUT_DISP_VALUE_TO_FILE 0
#endif

// Dump 3D point cloud to file.
#ifndef TQC_OUTPUT_3D_PCL_TO_FILE
#define TQC_OUTPUT_3D_PCL_TO_FILE 0
// Enable depth filter. If value's abs < FLT_EPSILON, discard it.
#if TQC_OUTPUT_3D_PCL_TO_FILE
#define TQC_SAVE_XYZ_FILTER 1
#else
#define TQC_SAVE_XYZ_FILTER 0
#endif
#endif

// Save disparity values as gray image file and color file.
#ifndef TQC_OUTPUT_DISP_TO_IMAGE
#define TQC_OUTPUT_DISP_TO_IMAGE 1
#endif

#ifndef TQC_STEREO_CAMERA_WIDTH
#define TQC_STEREO_CAMERA_WIDTH 320
#endif

#ifndef TQC_STEREO_CAMERA_HEIGHT
#define TQC_STEREO_CAMERA_HEIGHT 240
#endif

#ifndef TQC_STEREO_CAMERA_Y_BORDER
#define TQC_STEREO_CAMERA_Y_BORDER 60
#endif

#ifndef TQC_STEREO_CAMERA_X_BORDER
#define TQC_STEREO_CAMERA_X_BORDER 84
#endif

#ifdef WIN32
#define TQC_LOGICAL_CAM_LEFT_INDEX  1
#define TQC_LOGICAL_CAM_RIGHT_INDEX 0
#define TQC_INTERNAL_CAM_INDEX      2
#elif defined(ANDROID)
#define TQC_LOGICAL_CAM_LEFT_INDEX  0
#define TQC_LOGICAL_CAM_RIGHT_INDEX 1
#define TQC_INTERNAL_CAM_INDEX      2
#else
#define TQC_LOGICAL_CAM_LEFT_INDEX  0
#define TQC_LOGICAL_CAM_RIGHT_INDEX 1
#define TQC_INTERNAL_CAM_INDEX      2
#endif

// Virtual Copter's depth is 2m, Its real size is 30cm X 30cm. The image size is 56x56 in 320x240 source image.
#ifndef TQC_VIRTUAL_COPTER_X_SIZE
#define TQC_VIRTUAL_COPTER_X_SIZE 56
#endif

#ifndef TQC_VIRTUAL_COPTER_Y_SIZE
#define TQC_VIRTUAL_COPTER_Y_SIZE 56
#endif

// If enabling culling, the real width of image is (width - x_border * 2) and the real height of image is (height - y_border * 2).
#if TQC_STEREO_CULL
#define TQC_VIRTUAL_COPTER_LEFT   ((TQC_STEREO_CAMERA_WIDTH - TQC_STEREO_CAMERA_X_BORDER * 2) / 2 - TQC_VIRTUAL_COPTER_X_SIZE / 2)
#define TQC_VIRTUAL_COPTER_TOP    ((TQC_STEREO_CAMERA_HEIGHT - TQC_STEREO_CAMERA_Y_BORDER * 2) / 2 - TQC_VIRTUAL_COPTER_Y_SIZE / 2)
#define TQC_VIRTUAL_COPTER_RIGHT  ((TQC_STEREO_CAMERA_WIDTH - TQC_STEREO_CAMERA_X_BORDER * 2) / 2 + TQC_VIRTUAL_COPTER_X_SIZE / 2)
#define TQC_VIRTUAL_COPTER_BOTTOM ((TQC_STEREO_CAMERA_HEIGHT - TQC_STEREO_CAMERA_Y_BORDER * 2) / 2 + TQC_VIRTUAL_COPTER_Y_SIZE / 2)
#else
#define TQC_VIRTUAL_COPTER_LEFT   (TQC_STEREO_CAMERA_WIDTH / 2 - TQC_VIRTUAL_COPTER_X_SIZE / 2)
#define TQC_VIRTUAL_COPTER_TOP    (TQC_STEREO_CAMERA_HEIGHT / 2 - TQC_VIRTUAL_COPTER_Y_SIZE / 2)
#define TQC_VIRTUAL_COPTER_RIGHT  (TQC_STEREO_CAMERA_WIDTH / 2 + TQC_VIRTUAL_COPTER_X_SIZE / 2)
#define TQC_VIRTUAL_COPTER_BOTTOM (TQC_STEREO_CAMERA_HEIGHT / 2 + TQC_VIRTUAL_COPTER_Y_SIZE / 2)
#endif

#define TQC_VIRTUAL_COPTER_X_SPLITE 3
#define TQC_VIRTUAL_COPTER_Y_SPLITE TQC_VIRTUAL_COPTER_X_SPLITE
#define TQC_VIRTUAL_COPTER_SUB_X    TQC_VIRTUAL_COPTER_X_SIZE / 3
#define TQC_VIRTUAL_COPTER_SUB_Y    TQC_VIRTUAL_COPTER_Y_SIZE / 3
#define TQC_VIRTUAL_COPTER_SUB_X1   TQC_VIRTUAL_COPTER_LEFT + TQC_VIRTUAL_COPTER_X_SIZE / 3
#define TQC_VIRTUAL_COPTER_SUB_X2   TQC_VIRTUAL_COPTER_SUB_X1 + TQC_VIRTUAL_COPTER_X_SIZE / 3
#define TQC_VIRTUAL_COPTER_SUB_Y1   TQC_VIRTUAL_COPTER_TOP + TQC_VIRTUAL_COPTER_Y_SIZE / 3
#define TQC_VIRTUAL_COPTER_SUB_Y2   TQC_VIRTUAL_COPTER_SUB_Y1 + TQC_VIRTUAL_COPTER_Y_SIZE / 3

#endif /* __CONFIG_H */