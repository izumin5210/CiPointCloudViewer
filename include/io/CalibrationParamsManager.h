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
    std::string serial;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Mat r_mat;
    cv::Mat t;
    Eigen::Matrix4f calib_matrix;
    float fx, fy, cx, cy;
};

class CalibrationParamsManager {
public:
    typedef typename cinder::signals::Signal<void (const CalibrationParams&)> EventSignalCalibrationParams;

    static void load(std::string path, std::string prefix = "kinect2_");

    static EventSignalCalibrationParams& getSignalCalibrationParamsUpdated() {
        return get()->signal_calibration_params_updated_;
    }

    static CalibrationParamsManager* get() {
      static CalibrationParamsManager *manager = 0;
      if(!manager) {
        manager = new CalibrationParamsManager;
      }
      return manager;
    }


protected:
    EventSignalCalibrationParams signal_calibration_params_updated_;

    CalibrationParamsManager() {}
    ~CalibrationParamsManager() {}

    static CalibrationParams createCalibrationParams(std::string serial, cv::FileNode node);
};

}

#endif /* CalibrationParamsManager_h */
