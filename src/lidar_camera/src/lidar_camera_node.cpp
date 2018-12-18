#include "lidar_camera_node.h"
#include "colormap.h"
#include "calibration_solver.h"


#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>


using namespace sensor_msgs;
using namespace message_filters;

pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
Eigen::Matrix4f cooridnate_transfrom = Eigen::Matrix4f::Zero();


//TODO create a class and make these members
std::vector<double> params = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //x y z rx ry rz (rad)
std::vector<cv::Point2f> image_points; //
std::vector<cv::Point3d> lidar_points; //


bool done = false;

bool detectCheckerboardCenter(const cv::Mat &image, cv::Point2f *point){
    //Detect checkerboard in image
    std::vector<cv::Point2f> corners;
    cv::Size board_size(7,5);

    if(cv::findChessboardCorners(image, board_size, corners, CV_CALIB_CB_ADAPTIVE_THRESH)) {
        cv::Mat mean_mat;
        cv::reduce(corners, mean_mat, 01, CV_REDUCE_AVG);
        *point = cv::Point2f(mean_mat.at<float>(0,0), mean_mat.at<float>(0,1));
        return true;
    }

    return false;
}



void callback(const ImageConstPtr& image_msg, const CameraInfoConstPtr& cam_info_msg, const PointCloud2ConstPtr& lidar_msg) {

    cv::Mat image = cv_bridge::toCvShare(image_msg, image_encodings::BGR8)->image;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*lidar_msg, *cloud);

    //Transform the cloud into the correct cordinate system for cam model to be applied later
    pcl::transformPointCloud(*cloud, *cloud, cooridnate_transfrom);


    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

    //Filter points so we just see the person holding the board
    pcl::CropBox<pcl::PointXYZI> box_filter;
    box_filter.setInputCloud (cloud);
    box_filter.setMin(Eigen::Vector4f(-2, -2, 0, 1.0));
    box_filter.setMax(Eigen::Vector4f(2, 2, 2, 1.0));
    box_filter.setInputCloud(cloud);
    box_filter.filter(*cloud_filtered);

    std::vector<int> inliers;

    pcl::PointCloud<pcl::PointXYZI>::Ptr board_points(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZI> (cloud_filtered));
    pcl::RandomSampleConsensus<pcl::PointXYZI> ransac (model_p);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(inliers);

    pcl::copyPointCloud<pcl::PointXYZI>(*cloud_filtered, inliers, *board_points);

    viewer.showCloud(board_points);

    //Reproject and draw on the image
    image_geometry::PinholeCameraModel cam_model; // init cam_model
    cam_model.fromCameraInfo(cam_info_msg);


    cv::Point2f image_checkerboard_center;
    if( detectCheckerboardCenter(image, &image_checkerboard_center) ){
        //if center is in ROI i.e. the

        cv::Point3d lidar_checkerboard_center;

        pcl::PointXYZI centroid;
        pcl::computeCentroid(*board_points, centroid);

        lidar_checkerboard_center.x = centroid.x;
        lidar_checkerboard_center.y = centroid.y;
        lidar_checkerboard_center.z = centroid.z;

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

    //Todo run solver on matching points

    if(image_points.size() > 10 && !done) {
        CalibrationSolver calibration_solver;

        calibration_solver.solveParameters(image_points, lidar_points, cam_model, &params);
        done = true;

        for (const auto &val : params){
            std::cout << val << " ";
        }
        std::cout << std::endl;

    }

    //TODO transform cloud and send as message stream
    // rotate cloud http://www.pcl-users.org/Rotate-point-cloud-around-it-s-origin-td4040578.html

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.linear() = (Eigen::Matrix3f) Eigen::AngleAxisf(params[3], Eigen::Vector3f::UnitX())
                         * Eigen::AngleAxisf(params[4], Eigen::Vector3f::UnitY())
                         * Eigen::AngleAxisf(params[5], Eigen::Vector3f::UnitZ());
    transform.translation() << params[0], params[1], params[2];
    pcl::transformPointCloud(*board_points, *board_points, transform);





    for( int i = 0; i < board_points->size(); i++ ) {
        pcl::PointXYZI pt = board_points->points[i];

        cv::Point3d pt_cv(pt.x, pt.y, pt.z);
        cv::Point2d uv = cam_model.project3dToPixel(pt_cv);
        cv::circle(image, uv, 3, jetColormap(pt.intensity), -1);

    }

    for(int i = 0; i < image_points.size(); i++  ){

        cv::circle(image, image_points[i], 5, cv::Scalar(255, 255, 0), -1);
        cv::Point2d uv = cam_model.project3dToPixel(lidar_points[i]);
        cv::circle(image, uv, 5, cv::Scalar(0,255,255), -1);
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