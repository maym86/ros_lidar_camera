#include "lidar_camera_node.h"

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
   //https://answers.ros.org/question/136916/conversion-from-sensor_msgspointcloud2-to-pclpointcloudt/

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    viewer.showCloud(cloud);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv::Mat image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
        cv::imshow("view", image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert to image!");
    }
}



int main(int argc, char **argv) {
	ros::init(argc, argv, "lidar_camera");

	ros::NodeHandle node_handle;
    cv::namedWindow("view");
    cv::startWindowThread();

    ros::Subscriber image_sub = node_handle.subscribe("/sensors/camera/image_rect", 2, imageCallback);
    ros::Subscriber lidar_sub = node_handle.subscribe("/sensors/velodyne_points", 2, lidarCallback);

    ros::spin();
    cv::destroyWindow("view");


}