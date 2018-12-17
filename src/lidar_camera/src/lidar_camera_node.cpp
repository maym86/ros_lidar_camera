#include "lidar_camera_node.h"

#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/crop_box.h>

pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

using namespace sensor_msgs;
using namespace message_filters;

Eigen::Matrix4f cooridnate_transfrom = Eigen::Matrix4f::Zero();


void callback(const ImageConstPtr& image_msg, const CameraInfoConstPtr& cam_info_msg, const PointCloud2ConstPtr& lidar_msg) {



    cv::Mat image = cv_bridge::toCvShare(image_msg, image_encodings::BGR8)->image;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*lidar_msg, *cloud);


    pcl::transformPointCloud(*cloud, *cloud, cooridnate_transfrom);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);


    //Filter points so we just see the person holding the board
    pcl::CropBox<pcl::PointXYZI> box_filter;
    box_filter.setInputCloud (cloud);
    box_filter.setMin(Eigen::Vector4f(-2, -2, 0, 1.0)); //x forward back // y left right // z up down
    box_filter.setMax(Eigen::Vector4f(2, 2, 2, 1.0));
    box_filter.setInputCloud(cloud);
    box_filter.filter(*cloud_filtered);
    viewer.showCloud(cloud_filtered);

    //Reproject and draw on the image???
    image_geometry::PinholeCameraModel cam_model; // init cam_model
    cam_model.fromCameraInfo(cam_info_msg);


    for( int i = 0; i < cloud_filtered->size(); i++ ) {
        pcl::PointXYZI pt = cloud_filtered->points[i];

        cv::Point3d pt_cv(pt.x, pt.y, pt.z);
        cv::Point2d uv = cam_model.project3dToPixel(pt_cv);
        cv::circle(image, uv, 3, CV_RGB(255, 0, 0), -1);

    }
    cv::imshow("view", image);
    cv::waitKey(1);


}



int main(int argc, char **argv) {
	ros::init(argc, argv, "lidar_camera");

	ros::NodeHandle node_handle;
    cv::namedWindow("view");
    cv::startWindowThread();

    Subscriber<Image> image_sub(node_handle, "/sensors/camera/image_rect", 2);
    Subscriber<CameraInfo> info_sub(node_handle, "/sensors/camera/camera_info", 2);
    Subscriber<PointCloud2> lidar_sub(node_handle, "/sensors/velodyne_points", 2);

    typedef sync_policies::ApproximateTime<Image, CameraInfo, PointCloud2> SyncPolicy;
    Synchronizer<SyncPolicy> sync(SyncPolicy(10), image_sub, info_sub, lidar_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));


    cooridnate_transfrom(0,1) = -1;
    cooridnate_transfrom(1,2) = -1;
    cooridnate_transfrom(2,0) = 1;
    cooridnate_transfrom(3,3) = 1;


    ros::spin();
    cv::destroyWindow("view");
}