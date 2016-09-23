//
// Created by Masayuki IZUMI on 2016/09/08.
//


#include <OpenNI.h>
#include "opencv2/opencv.hpp"

#include "impl/io/SensorDeviceManagerImpl.h"
#include "io/SensorDevice.hpp"
#include "io/CalibrationParamsManager.h"

namespace io {

class SensorDeviceManagerImpl : public  SensorDeviceManager {
public:
  INJECT(SensorDeviceManagerImpl(
      const std::shared_ptr<Dispatcher> dispatcher
  ))
    : dispatcher_(dispatcher)
  {
    openni::OpenNI::initialize();
  }

  ~SensorDeviceManagerImpl() {
    stop();
    openni::OpenNI::shutdown();
  }

  std::map<std::string, std::shared_ptr<SensorDevice>> devices() override {
    return devices_;
  }

  void start() override {
    stopped_ = false;
    refresh();
  }

  void stop() override {
    stopped_ = true;
    if (device_check_worker_.joinable()) {
      device_check_worker_.join();
    }
    for (auto pair : devices_) {
      pair.second->stop();
    }
  }

  void refresh() override {
    openni::Array<openni::DeviceInfo> device_info_list;
    openni::OpenNI::enumerateDevices(&device_info_list);
    for (int i = 0; i < device_info_list.getSize(); i += 3) {
      std::string uri = device_info_list[i].getUri();
      if (devices_.find(uri) == devices_.end()) {
        open(uri);
      }
    }
  }

  void open(const std::string uri) override {
    auto device = std::make_shared<SensorDevice>(dispatcher_);
    device->initialize(uri.c_str());
    devices_[uri] = device;
  }

private:
  const std::shared_ptr<Dispatcher> dispatcher_;
  std::map<std::string, std::shared_ptr<SensorDevice>> devices_;
  std::vector<CalibrationParams> calib_params_list_;
  std::atomic<bool> stopped_;
  std::thread device_check_worker_;
};

}

fruit::Component<fruit::Required<Dispatcher>, io::SensorDeviceManager>
getSensorDeviceManagerImplComponent() {
  return fruit::createComponent()
      .bind<io::SensorDeviceManager, io::SensorDeviceManagerImpl>();
}
