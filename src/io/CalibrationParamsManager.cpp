//
//  CalibrationParams.cpp
//  CiPointCloudViewer
//
//  Created by Masayuki IZUMI on 11/8/16.
//
//

#ifndef CalibrationParams_cpp
#define CalibrationParams_cpp

#include "Clouds.h"
#include "Signal.h"
#include "io/CalibrationParamsManager.h"

namespace io {

void
CalibrationParams::load(std::string path, std::string prefix) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    for (auto node : fs.root()) {
        auto serial = node.name().substr(strlen(prefix.c_str()));
        auto params = createCalibrationParams(serial, node);
        Signal<Clouds::UpdateCalibrationParamsAction>::emit({serial, params});
    }
    fs.release();
}

CalibrationParams
CalibrationParams::createCalibrationParams(std::string serial, cv::FileNode node) {
    cv::Mat camera_matrix (cv::Mat::eye(3, 3, CV_64FC1));
    cv::Mat dist_coeffs   (5, 1, CV_64FC1, cv::Scalar::all(0));
    cv::Mat r_mat         (3, 3, CV_64FC1, cv::Scalar::all(0));
    cv::Mat t             (3, 1, CV_64FC1, cv::Scalar::all(0));

    node["camera_matrix"]           >> camera_matrix;
    node["distortion_coefficients"] >> dist_coeffs;
    node["rotation"]                >> r_mat;
    node["translation"]             >> t;

    float fx = camera_matrix.ptr<double>(0)[0];
    float fy = camera_matrix.ptr<double>(0)[4];
    float cx = camera_matrix.ptr<double>(0)[2];
    float cy = camera_matrix.ptr<double>(0)[5];

    Eigen::Matrix4f tmp_mat = Eigen::Matrix4f::Identity();

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            tmp_mat(i, j) = r_mat.at<double>(i, j);
        }
    }

    for (int i = 0; i < 3; i++) {
        tmp_mat(i, 3) = t.at<double>(i);
    }

    return {
        serial,
        camera_matrix,
        dist_coeffs,
        r_mat,
        t,
        tmp_mat.inverse(),
        fx,
        fy,
        cx,
        cy
    };
}

}

#endif /* CalibrationParams_cpp */
