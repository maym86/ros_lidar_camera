
#include "process_camera.h"

#include <opencv2/calib3d/calib3d.hpp>


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