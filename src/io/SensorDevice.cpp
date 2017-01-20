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
{}

SensorDevice::~SensorDevice()
{
  cloud_data_source_.release();
}

void SensorDevice::initialize(const char *uri) {
  uri_ = uri;

  openni::Device device;

  auto status = device.open(uri);
  if (status != openni::STATUS_OK) {
    throw std::runtime_error("openni::Device::open() failed.");
  }

  if (!device.isFile()) {
    char serial[64];
    status = device.getProperty(ONI_DEVICE_PROPERTY_SERIAL_NUMBER, &serial);
    if (status != openni::STATUS_OK) {
      throw std::runtime_error("Device has not serial number.");
    }
    serial_.assign(serial, strlen(serial));
    name_ = serial_;
  }

  device.close();

  cloud_data_source_ = std::unique_ptr<datasource::OpenNI2CloudDataSource>(new datasource::OpenNI2CloudDataSource(name_, uri_));
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
