
#include "process_lidar.h"

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl_conversions/pcl_conversions.h>

ProcessLidar::ProcessLidar(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud){
    cloud_ = cloud;
    filterROI();

    x_min_ = -2;
    x_max_ = 2;
    y_min_ = -2;
    y_max_ = 2;
    z_min_ = 0;
    z_max_ = 2;
}

void ProcessLidar::setROI(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max){
    x_min_ = x_min;
    x_max_ = x_max;
    y_min_ = y_min;
    y_max_ = y_max;
    z_min_ = z_min;
    z_max_ = z_max;
}


void  ProcessLidar::filterROI(){
    cloud_filtered_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

    //Filter points so we just see the person holding the board
    pcl::CropBox<pcl::PointXYZI> box_filter;
    box_filter.setInputCloud (cloud_);
    box_filter.setMin(Eigen::Vector4f(x_min_, y_min_, z_min_, 1.0));
    box_filter.setMax(Eigen::Vector4f(x_max_, y_max_, z_max_, 1.0));
    box_filter.setInputCloud(cloud_);
    box_filter.filter(*cloud_filtered_);
}

bool ProcessLidar::detectCheckerboard(int min_board_points, cv::Point3d *lidar_checkerboard_center){

    board_points_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

    std::vector<int> inliers;

    pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZI> (cloud_filtered_));
    pcl::RandomSampleConsensus<pcl::PointXYZI> ransac (model_p);
    ransac.setDistanceThreshold (0.01);
    ransac.computeModel();
    ransac.getInliers(inliers);

    pcl::copyPointCloud<pcl::PointXYZI>(*cloud_filtered_, inliers, *board_points_);

    pcl::PointXYZI centroid;
    pcl::computeCentroid(*board_points_, centroid);


    if(board_points_->size() > min_board_points) {
        lidar_checkerboard_center->x = centroid.x;
        lidar_checkerboard_center->y = centroid.y;
        lidar_checkerboard_center->z = centroid.z;


        return true;
    }
    return false;
}