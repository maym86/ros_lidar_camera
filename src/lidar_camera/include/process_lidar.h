
#ifndef LIDAR_CAMERA_LIDAR_H
#define LIDAR_CAMERA_LIDAR_H

#include <cxcore.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

class ProcessLidar {

public:

    ProcessLidar(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);


    bool detectCheckerboard(cv::Point3d *lidar_checkerboard_center);

    void setROI(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);

    void filterROI();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr board_points_;
private:


    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;

    float x_min_;
    float x_max_;
    float y_min_;
    float y_max_;
    float z_min_;
    float z_max_;
};

#endif //LIDAR_CAMERA_LIDAR_H
