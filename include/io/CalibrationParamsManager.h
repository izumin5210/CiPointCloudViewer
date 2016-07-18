//
//  CalibrationParamsManager.h
//  CiPointCloudViewer
//
//  Created by Masayuki IZUMI on 11/8/16.
//
//

#ifndef CalibrationParamsManager_h
#define CalibrationParamsManager_h

#include "cinder/Signals.h"

#include <Eigen/Core>
#include <Eigen/LU>

#include <opencv2/opencv.hpp>

namespace io {

struct CalibrationParams {
public:
  std::string serial;
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  cv::Mat r_mat;
  cv::Mat t;
  Eigen::Matrix4f calib_matrix;
  float fx, fy, cx, cy;

  static void load(std::string path, std::string prefix = "kinect2_");


private:
  static CalibrationParams createCalibrationParams(std::string serial, cv::FileNode node);
};

}

#endif /* CalibrationParamsManager_h */
