//
//  SensorDevice.hpp
//  CiPointCloudViewer
//
//  Created by Masayuki IZUMI on 7/8/16.
//
//

#include "io/SensorDevice.h"
#include "io/datasource/OpenNI2CloudDataSource.h"

namespace io {

SensorDevice::SensorDevice()
  : device_(new openni::Device)
{}

SensorDevice::~SensorDevice() {
  stop();
  device_->close();
}

void SensorDevice::initialize(const char *uri) {
  uri_ = uri;

  auto status = device_->open(uri);
  if (status != openni::STATUS_OK) {
    throw std::runtime_error("openni::Device::open() failed.");
  }

  if (!device_->isFile()) {
    char serial[64];
    status = device_->getProperty(ONI_DEVICE_PROPERTY_SERIAL_NUMBER, &serial);
    if (status != openni::STATUS_OK) {
      throw std::runtime_error("Device has not serial number.");
    }
    serial_.assign(serial, strlen(serial));
    name_ = serial_;
  }

  cloud_data_source_ = std::make_unique<datasource::OpenNI2CloudDataSource>(name_, device_);
}

void SensorDevice::start() {
  cloud_data_source_->startFetching();
}

void SensorDevice::stop() {
  cloud_data_source_->stopFetching();
}

void SensorDevice::record(std::string dir) {
  cloud_data_source_->startRecording(dir);
}

void SensorDevice::stopRecording() {
  cloud_data_source_->stopRecording();
}

}
