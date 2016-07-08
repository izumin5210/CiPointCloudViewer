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
#include "CalibrationParams.hpp"

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
        std::cout << "Add " << device->serial() << std::endl;
        devices_[uri] = device;
    }

    inline void loadCalibrationMatrix(std::string path, std::string prefix = "kinect2_") {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        for (auto node : fs.root()) {
          std::string serial = node.name().substr(strlen(prefix.c_str()));
            // calib_params_list_[serial] = std::make_shared<CalibrationParams>();
            // calib_params_list_[serial]->initialize(node);
            CalibrationParams params;
            params.initialize(node);
            std::cout << serial << std::endl;
            for (auto pair : devices_) {
              if (pair.second->serial() == serial) {
                std::cout << "exists" << std::endl;
                // devices_[serial]->setCalibrationParams(calib_params_list_[serial]);
                devices_[serial]->setCalibrationParams(params);
                devices_[serial]->start([this]() { update(); });
              }
            }
        }
        fs.release();
    }

    inline pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud() {
      return cloud_;
    }

private:
    std::map<std::string, std::shared_ptr<SensorDevice>> devices_;
    std::map<std::string, std::shared_ptr<CalibrationParams>> calib_params_list_;
    std::atomic<bool> stopped_;
    std::thread device_check_worker_;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;

    inline void update() {
      cloud_->clear();
      for (auto pair : devices_) {
        *cloud_ += *pair.second->cloud();
      }
    }
};

}

#endif /* SensorDeviceManager_hpp */
