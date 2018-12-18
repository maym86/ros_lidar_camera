#include "ros/ros.h"

#include "colormap.h"
#include "calibration_solver.h"
#include "process_lidar.h"
#include "process_camera.h"

#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

using namespace sensor_msgs;
using namespace message_filters;


const int MIN_REQUIRED_POINTS = 10;

Eigen::Matrix4f cooridnate_transfrom = Eigen::Matrix4f::Zero();


//TODO create a class and make these members
std::vector<double> params = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //x y z rx ry rz (rad)
std::vector<cv::Point2f> image_points; //
std::vector<cv::Point3d> lidar_points; //


bool solved = false;

void callback(const ImageConstPtr& image_msg, const CameraInfoConstPtr& cam_info_msg, const PointCloud2ConstPtr& lidar_msg) {

    cv::Mat image = cv_bridge::toCvShare(image_msg, image_encodings::BGR8)->image;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*lidar_msg, *cloud);

    //Transform the cloud into the correct cordinate system for cam model to be applied later
    pcl::transformPointCloud(*cloud, *cloud, cooridnate_transfrom);

    //Reproject and draw on the image
    image_geometry::PinholeCameraModel cam_model; // init cam_model
    cam_model.fromCameraInfo(cam_info_msg);

    ProcessLidar process_lidar(cloud);
    cv::Point2f image_checkerboard_center;
    if( detectCheckerboardCenter(image, &image_checkerboard_center) ){
        //if center is in ROI i.e. the


        cv::Point3d lidar_checkerboard_center;
        process_lidar.detectCheckerboard(&lidar_checkerboard_center);

        bool skip = false;
        for(auto point: lidar_points){
            if(cv::norm(point - lidar_checkerboard_center) < 0.25){
                skip = true;
            }
        }

        if(!skip){
            image_points.push_back(std::move(image_checkerboard_center));
            lidar_points.push_back(std::move(lidar_checkerboard_center));
        }

    }

    //If there have been enough points collected run the solver
    if(image_points.size() > MIN_REQUIRED_POINTS && !solved) {
        CalibrationSolver calibration_solver;

        calibration_solver.solveParameters(image_points, lidar_points, cam_model, &params);
        solved = true;

        for (const auto &val : params){
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }

    //Transform the cloud to show the result
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.linear() = (Eigen::Matrix3f) Eigen::AngleAxisf(params[3], Eigen::Vector3f::UnitX())
                         * Eigen::AngleAxisf(params[4], Eigen::Vector3f::UnitY())
                         * Eigen::AngleAxisf(params[5], Eigen::Vector3f::UnitZ());
    transform.translation() << params[0], params[1], params[2];

    pcl::PointCloud<pcl::PointXYZI>::Ptr roateted_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*process_lidar.cloud_filtered_, *roateted_cloud, transform);

    //Draw result
    for( int i = 0; i < roateted_cloud->size(); i++ ) {
        pcl::PointXYZI pt = roateted_cloud->points[i];
        cv::Point3d pt_cv(pt.x, pt.y, pt.z);
        cv::Point2d uv = cam_model.project3dToPixel(pt_cv);
        cv::circle(image, uv, 3, jetColormap(pt.intensity), -1);
    }

    //Draw points that are used for matching
    if(!solved) {
        cv::Point2f text_offset(10, 0);
        for (int i = 0; i < image_points.size(); i++) {
            cv::circle(image, image_points[i], 5, cv::Scalar(255, 0, 0), -1);
            cv::Point2f uv = cam_model.project3dToPixel(lidar_points[i]);
            cv::circle(image, uv, 5, cv::Scalar(0, 0, 255), -1);
            cv::putText(image, std::to_string(i+1), image_points[i] + text_offset, 1, 1, cv::Scalar(255, 0, 0));
            cv::putText(image, std::to_string(i+1), uv + text_offset, 1, 1, cv::Scalar(0, 0, 255));
        }
    }

    cv::imshow("view", image);
    cv::waitKey(1);


    //TODO transform cloud and send as message stream

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