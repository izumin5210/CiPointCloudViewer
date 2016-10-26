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
#include <opencv2/opencv.hpp>

#ifdef USE_NITE2
#include <NiTE.h>
#endif

#include "io/SensorDevice.h"
#include "io/CalibrationParamsManager.h"
#include "Signal.h"

namespace io {

class SensorDeviceManager {
public:
    SensorDeviceManager() {
        openni::OpenNI::initialize();
#ifdef USE_NITE2
        if (nite::NiTE::initialize() != nite::STATUS_OK) {
            throw std::runtime_error("Failed to initialize NiTE2.");
        }
#endif
        stopped_ = false;
        refresh();
    }

    ~SensorDeviceManager() {
        stopped_ = true;
        if (device_check_worker_.joinable()) {
            device_check_worker_.join();
        }
#ifdef USE_NITE2
        nite::NiTE::shutdown();
#endif
        openni::OpenNI::shutdown();
    }

    inline std::map<std::string, std::shared_ptr<SensorDevice>> devices() {
        return devices_;
    }

    inline void refresh() {
        openni::Array<openni::DeviceInfo> device_info_list;
        openni::OpenNI::enumerateDevices(&device_info_list);
        for (int i = 0; i < device_info_list.getSize(); i += 3) {
            std::string uri = device_info_list[i].getUri();
            if (devices_.find(uri) == devices_.end()) {
                open(uri);
            }
        }
    }

    inline void open(const std::string uri) {
        auto device = std::make_shared<SensorDevice>();
        device->initialize(uri.c_str());
        devices_[uri] = device;
    }


private:
    std::map<std::string, std::shared_ptr<SensorDevice>> devices_;
    std::vector<CalibrationParams> calib_params_list_;
    std::atomic<bool> stopped_;
    std::thread device_check_worker_;
};

}

#endif /* SensorDeviceManager_hpp */
