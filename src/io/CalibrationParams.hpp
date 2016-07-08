//
//  CalibrationParams.hpp
//  CiPointCloudViewer
//
//  Created by Masayuki IZUMI on 7/8/16.
//
//

#ifndef CalibrationParams_hpp
#define CalibrationParams_hpp

#include <Eigen/Core>
#include <Eigen/LU>

#include <opencv2/opencv.hpp>

namespace io {

class CalibrationParams {
public:
    CalibrationParams()
        : camera_matrix_(cv::Mat::eye(3, 3, CV_64FC1))
        , dist_coeffs_  (5, 1, CV_64FC1, cv::Scalar::all(0))
        , r_mat_        (3, 3, CV_64FC1, cv::Scalar::all(0))
        , t_            (3, 1, CV_64FC1, cv::Scalar::all(0))
    {
    }

    inline void initialize(cv::FileNode &node) {
        node["camera_matrix"]           >> camera_matrix_;
        node["distortion_coefficients"] >> dist_coeffs_;
        node["rotation"]                >> r_mat_;
        node["translation"]             >> t_;

        fx_ = camera_matrix_.ptr<double>(0)[0];
        fy_ = camera_matrix_.ptr<double>(0)[4];
        cx_ = camera_matrix_.ptr<double>(0)[2];
        cy_ = camera_matrix_.ptr<double>(0)[5];

        Eigen::Matrix4f tmp_mat = Eigen::Matrix4f::Identity();

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                tmp_mat(i, j) = r_mat_.at<double>(i, j);
            }
        }

        for (int i = 0; i < 3; i++) {
            tmp_mat(i, 3) = t_.at<double>(i);
        }

        calib_matrix_ = tmp_mat.inverse();
    }

private:
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat r_mat_;
    cv::Mat t_;
    Eigen::Matrix4f calib_matrix_;
    float fx_, fy_, cx_, cy_;
};

}

#endif /* CalibrationParams_hpp */
