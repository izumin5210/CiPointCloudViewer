//
//  SensorDeviceManager.hpp
//  CiPointCloudViewer
//
//  Created by Masayuki IZUMI on 7/8/16.
//
//

#ifndef SensorDeviceManager_hpp
#define SensorDeviceManager_hpp

#include <OpenNI.h>
#include "opencv2/opencv.hpp"

#include "SensorDevice.hpp"
#include "io/CalibrationParamsManager.h"

namespace io {

class SensorDeviceManager {
public:
    SensorDeviceManager()
      : cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>())
    {
        openni::OpenNI::initialize();
    }

    ~SensorDeviceManager() {
        stop();
        openni::OpenNI::shutdown();
    }

    inline std::map<std::string, std::shared_ptr<SensorDevice>> devices() {
        return devices_;
    }

    inline void start() {
        stopped_ = false;
        CalibrationParamsManager::getSignalCalibrationParamsUpdated()
          .connect(ci::signals::slot(this, &SensorDeviceManager::addCalibrationParams));
        device_check_worker_ = std::thread([this]() {
            openni::Array<openni::DeviceInfo> device_info_list;
            while (!stopped_) {
                openni::OpenNI::enumerateDevices(&device_info_list);
                for (int i = 0; i < device_info_list.getSize(); i += 3) {
                    std::string uri = device_info_list[i].getUri();
                    if (devices_.find(uri) == devices_.end()) {
                        open(uri);
                    }
                }
            }
        });
    }

    inline void stop() {
        stopped_ = true;
        if (device_check_worker_.joinable()) {
            device_check_worker_.join();
        }
    }

    inline void open(const std::string uri) {
        auto device = std::make_shared<SensorDevice>();
        device->initialize(uri.c_str());
        devices_[uri] = device;
    }

    inline pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud() {
      std::lock_guard<std::mutex> lg(mutex_);
      return cloud_;
    }


private:
    std::map<std::string, std::shared_ptr<SensorDevice>> devices_;
    std::vector<CalibrationParams> calib_params_list_;
    std::atomic<bool> stopped_;
    std::thread device_check_worker_;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;

    std::mutex mutex_;

    std::function<void()> on_updated_ = [this]() {
      std::lock_guard<std::mutex> lg(mutex_);
      cloud_->clear();
      for (auto pair : devices_) {
        if (!pair.second->cloud()->empty()) {
          *cloud_ += *(pair.second->cloud());
        }
      }
    };

    void addCalibrationParams(CalibrationParams params) {
        calib_params_list_.push_back(params);
    }
};

}

#endif /* SensorDeviceManager_hpp */
