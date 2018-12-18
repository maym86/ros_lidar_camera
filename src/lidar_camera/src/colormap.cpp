#include "colormap.h"

double interpolate(double val, double y0, double x0, double y1, double x1) {
    return (val - x0) * (y1 - y0) / (x1 - x0) + y0;
}

double getIntensityValue(double val) {
    if (val <= -0.75) {
        return 0;
    } else if (val <= -0.25) {
        return interpolate(val, 0.0, -0.75, 1.0, -0.25);
    } else if (val <= 0.25) {
        return 1.0;
    } else if (val <= 0.75) {
        return interpolate(val, 1.0, 0.25, 0.0, 0.75);
    } else {
        return 0.0;
    }
}

double red(double gray) {
    return getIntensityValue(gray - 0.5);
}

double green(double gray) {
    return getIntensityValue(gray);
}

double blue(double gray) {
    return getIntensityValue(gray + 0.5);
}

cv::Scalar jetColormap(float intensity) {
    if (intensity > 255) {  //white
        return cv::Scalar(255, 255, 255);
    }

    float r = red(intensity / 256.0);
    float g = green(intensity / 256.0);
    float b = blue(intensity / 256.0);

    return cv::Scalar(b * 256, g * 256, r * 256);
}
