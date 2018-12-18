
#include "solver.h"
#include "utils.h"

struct ReprojectionError {

    ReprojectionError(const cv::Point2f &image_point, const cv::Point3d &lidar_point, const image_geometry::PinholeCameraModel &cam_model) {
        image_point_ = image_point;
        lidar_point_ = lidar_point;
        cam_model_ = cam_model;
    }

    bool operator()(const double *params_in, double *residual) const {
        //transform points and calc residual
        cv::Mat r_mat =  eulerAnglesToRotationMatrix(params_in[3], params_in[4], params_in[5], false);
        cv::Mat rotated = r_mat * cv::Mat(lidar_point_);

        //TODO double check that access is correct here might be (0,1) etc.

        cv::Point3d lidar_point(rotated);
        lidar_point.x += params_in[0];
        lidar_point.y += params_in[1];
        lidar_point.z += params_in[2];

        //Transform point to image coordinates using camerainfo and compute residuals
        cv::Point2d uv = cam_model_.project3dToPixel(lidar_point);

        residual[0] = uv.x - image_point_.x;
        residual[1] = uv.y - image_point_.y;
        return true;
    }

  private:
    image_geometry::PinholeCameraModel cam_model_;
    cv::Point2f image_point_;
    cv::Point3d lidar_point_;

};

void Solver::solveParameters(const std::vector<cv::Point2f> &image_points,
                                        const std::vector<cv::Point3d> &lidar_points,
                                        const image_geometry::PinholeCameraModel &cam_model,
                                        std::vector<double> *lidar_params) {
    ceres::Problem problem;


    double* solution = &lidar_params->front(); //TODO verify that this is ok and the double is pointing at the data

    for (size_t i = 0; i < image_points.size(); i++) {
        ceres::CostFunction *cost_function =
            new ceres::NumericDiffCostFunction<ReprojectionError, ceres::CENTRAL, 2, 6>(new ReprojectionError(
                image_points[i], lidar_points[i], cam_model));

        problem.AddResidualBlock(cost_function, NULL, solution);
    }

    // Run the solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;
    options.parameter_tolerance = 1e-10;
    options.max_num_iterations = 1000;

    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << std::endl;


}
