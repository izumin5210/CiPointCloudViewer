//
//  SensorDevice.hpp
//  CiPointCloudViewer
//
//  Created by Masayuki IZUMI on 7/8/16.
//
//

#ifndef SensorDevice_hpp
#define SensorDevice_hpp

#include <OpenNI.h>

namespace io {

class SensorDevice {
public:
    inline void initialize(const char *uri = openni::ANY_DEVICE) {
        uri_ = uri;

        checkStatus(device_.open(uri), "openni::Device::open() failed.");
        if (device_.isFile()) {
            // TODO: not yet implemented
        } else {
            char serial[64];
            auto status = device_.getProperty(ONI_DEVICE_PROPERTY_SERIAL_NUMBER, &serial);
            checkStatus(status, "Device has not serial number.");
            serial_.assign(serial, strlen(serial));
        }
    }

    inline std::string uri() {
        return uri_;
    }

    inline std::string serial() {
        return serial_;
    }

    inline void setCalibrationParams(CalibrationParams& params) {
        params_ = params;
    }

private:
    openni::Device device_;
    std::string uri_;
    std::string serial_;
    CalibrationParams params_;

    inline void checkStatus(openni::Status status, std::string msg) {
        if (status != openni::STATUS_OK) { 
            throw std::runtime_error(msg);
        }
    }
};

}

#endif /* SensorDevice_hpp */
