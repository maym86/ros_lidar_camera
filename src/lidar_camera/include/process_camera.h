
#ifndef LIDAR_CAMERA_PROCESS_CAMERA_H
#define LIDAR_CAMERA_PROCESS_CAMERA_H

#include <cxcore.h>

bool detectCheckerboardCenter(const cv::Mat &image, cv::Point2f *point);

#endif //LIDAR_CAMERA_PROCESS_CAMERA_H
