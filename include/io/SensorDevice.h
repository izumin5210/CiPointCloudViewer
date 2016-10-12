//
// Created by Masayuki IZUMI on 9/28/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_SENSORDEVICE_H
#define CIPOINTCLOUDVIEWERAPP_SENSORDEVICE_H

#include <atomic>
#include <thread>

#include <OpenNI.h>

#include "datasource/CloudDataSource.h"

namespace io {

class SensorDevice {
public:
  SensorDevice();
  ~SensorDevice();

  void initialize(const char* url = openni::ANY_DEVICE);
  void start();
  void stop();
  void record(std::string dir);
  void stopRecording();

  std::string uri() const {
    return uri_;
  }

  std::string serial() const {
    return serial_;
  }

  float fps() const {
    return cloud_data_source_->fps();
  }

  std::string state() const {
    return cloud_data_source_->stateString();
  }

  bool isRunning() const {
    return cloud_data_source_->isRunning();
  }

  bool isRecording() const {
    return cloud_data_source_->isRecording();
  }


private:
  std::string uri_;
  std::string serial_;
  std::string name_;

  std::unique_ptr<datasource::CloudDataSource> cloud_data_source_;

  void checkStatus(openni::Status status, std::string msg);
};

}

#endif //CIPOINTCLOUDVIEWERAPP_SENSORDEVICE_H
