#include "utils.h"

cv::Mat eulerAnglesToRotationMatrix(double x, double y, double z, bool input_deg) {
    if (input_deg) {
        x *= (CV_PI / 180.0);
        y *= (CV_PI / 180.0);
        z *= (CV_PI / 180.0);
    }

    cv::Mat RX = (cv::Mat_<double>(3, 3) << 1, 0, 0,
            0, cos(x), -sin(x),
            0, sin(x), cos(x));

    cv::Mat RY = (cv::Mat_<double>(3, 3) << cos(y), 0, sin(y),
            0, 1, 0,
            -sin(y), 0, cos(y));

    cv::Mat RZ = (cv::Mat_<double>(3, 3) << cos(z), -sin(z), 0,
            sin(z), cos(z), 0,
            0, 0, 1);

    cv::Mat R = RZ * RY * RX;
    return R;
}