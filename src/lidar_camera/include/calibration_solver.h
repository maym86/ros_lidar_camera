
#ifndef LIDAR_CAMERA_SOLVER_H
#define LIDAR_CAMERA_SOLVER_H

#include "ceres/ceres.h"

#include <image_geometry/pinhole_camera_model.h>

class CalibrationSolver {
  public:
    void solveParameters(const std::vector<cv::Point2f> &image_points,
                         const std::vector<cv::Point3d> &lidar_points,
                         const image_geometry::PinholeCameraModel &cam_model,
                         std::vector<double> *lidar_params);
};

#endif  //LIDAR_CAMERA_SOLVER_H
