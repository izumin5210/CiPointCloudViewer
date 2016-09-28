//
// Created by Masayuki IZUMI on 9/28/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_SENSORDEVICE_H
#define CIPOINTCLOUDVIEWERAPP_SENSORDEVICE_H

#include <atomic>
#include <thread>

#include <opencv2/core.hpp>
#include <OpenNI.h>

#include "FpsCounter.h"
#include "Clouds.h"
#include "io/CalibrationParamsManager.h"

namespace io {

class SensorDevice {
public:
  enum State {
    NOT_INITIALIZED,
    INITIALIZED,
    CALIBRATED,
    STARTING,
    PLAYING,
    RECORDING,
    STOPPING
  };

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
    return fps_;
  }

  bool hasStarted() const {
    return !worker_canceled_;
  }

  bool hasCalibrationParams() const {
    return calibrated_;
  }

  bool isReady() const {
    return calibrated_ && worker_canceled_;
  }

  bool isRecording() const {
    return recording_;
  }

  State state() const {
    return state_;
  }

  std::string stateString() {
    return kStateString[state()];
  }

private:
  std::map<State, std::string> kStateString = {
      { NOT_INITIALIZED,  "NOT_INITIALIZED" },
      { INITIALIZED,      "INITIALIZED" },
      { CALIBRATED,       "CALIBRATED" },
      { STARTING,         "STARTING" },
      { PLAYING,          "PLAYING" },
      { RECORDING,        "RECORDING" },
      { STOPPING,         "STOPPING" }
  };

  openni::Device device_;
  std::string uri_;
  std::string serial_;
  std::string name_;
  CalibrationParams params_;

  FpsCounter fps_counter_;

  openni::VideoStream color_stream_;
  openni::VideoStream depth_stream_;
  openni::VideoStream ir_stream_;

  openni::Recorder recorder_;

  cv::Mat color_image_;
  cv::Mat raw_depth_image_;
  cv::Mat depth_image_;
  cv::Mat ir_image_;

  State state_;

  std::thread worker_;
  std::atomic<bool> worker_canceled_;
  std::atomic<bool> calibrated_;
  std::atomic<bool> recording_;

  float fps_;

  void setCalibrationParams(const Clouds::UpdateCalibrationParamsAction &action);
  void updateFps(const FpsCounter::Event& event);
  void checkStatus(openni::Status status, std::string msg);
  void startColorStream();
  void startDepthStream();
  void startIrStream();
  void enableMirroring();
  void enableDepthToColorRegistration();
  void update();
  void updateColorImage(const openni::VideoFrameRef &color_frame);
  void updateRawDepthImage(const openni::VideoFrameRef &depth_frame);
  void updateDepthImage(const openni::VideoFrameRef &depth_frame);
  void updateIrImage(const openni::VideoFrameRef &ir_frame);
  void updatePointCloud(std::chrono::system_clock::time_point timestamp);
};

}

#endif //CIPOINTCLOUDVIEWERAPP_SENSORDEVICE_H
