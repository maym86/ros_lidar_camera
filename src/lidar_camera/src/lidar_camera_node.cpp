#include "lidar_camera_node.h"

#include <ros/ros.h>

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

pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

using namespace sensor_msgs;
using namespace message_filters;


void callback(const ImageConstPtr& image_msg, const CameraInfoConstPtr& cam_info_msg, const PointCloud2ConstPtr& lidar_msg) {

    try {
        cv::Mat image = cv_bridge::toCvShare(image_msg, image_encodings::BGR8)->image;
        cv::imshow("view", image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert to image!");
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*lidar_msg, *cloud);
    viewer.showCloud(cloud);





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

    ros::spin();
    cv::destroyWindow("view");
}