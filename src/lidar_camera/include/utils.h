#ifndef LIDAR_CAMERA_UTILS_H
#define LIDAR_CAMERA_UTILS_H

#include <cxcore.h>

cv::Mat eulerAnglesToRotationMatrix(double x, double y, double z, bool input_deg);

#endif //LIDAR_CAMERA_UTILS_H
