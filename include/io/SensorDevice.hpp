//
//  SensorDevice.hpp
//  CiPointCloudViewer
//
//  Created by Masayuki IZUMI on 7/8/16.
//
//

#ifndef SensorDevice_hpp
#define SensorDevice_hpp

#include <sstream>
#include <algorithm>
#include <iostream>

#include <boost/date_time.hpp>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/utility.hpp>
#include <OpenNI.h>

#include "Clouds.h"
#include "FpsCounter.h"
#include "Signal.h"
#include "Vertex.h"
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

    SensorDevice()
      : state_(NOT_INITIALIZED)
      , worker_canceled_(true)
      , calibrated_(false)
      , recording_(false)
      , fps_(0.0f)
    {
    }

    ~SensorDevice() {
        stop();
        device_.close();
    }

    void initialize(const char *uri = openni::ANY_DEVICE) {
        uri_ = uri;

        namespace ph = std::placeholders;
        Signal<Clouds::UpdateCalibrationParamsAction>::connect(std::bind(&SensorDevice::setCalibrationParams, this, ph::_1));
        Signal<FpsCounter::Event>::connect(std::bind(&SensorDevice::updateFps, this, ph::_1));

        checkStatus(device_.open(uri), "openni::Device::open() failed.");
        if (device_.isFile()) {
            // TODO: not yet implemented
        } else {
            char serial[64];
            auto status = device_.getProperty(ONI_DEVICE_PROPERTY_SERIAL_NUMBER, &serial);
            // checkStatus(status, "Device has not serial number.");
            serial_.assign(serial, strlen(serial));

            name_ = serial_;
            // name_ = uri;
            // std::replace(name_.begin(), name_.end(), ':', ' ');
            // std::replace(name_.begin(), name_.end(), '@', ' ');
            // std::stringstream buf(name_);
            // buf >> name_;
        }

        state_ = INITIALIZED;
    }

    std::string uri() {
        return uri_;
    }

    std::string serial() {
        return serial_;
    }

    float fps() {
        return fps_;
    }

    bool hasStarted() {
        return !worker_canceled_;
    }

    bool hasCalibrationParams() {
        return calibrated_;
    }

    bool isReady() {
        return calibrated_ && worker_canceled_;
    }

    bool isRecording() {
        return recording_;
    }

    State state() {
        return state_;
    }

    std::string stateString() {
        return kStateString[state_];
    }

    void setCalibrationParams(const Clouds::UpdateCalibrationParamsAction &action) {
        if (action.key == serial_) {
            if (state_ == INITIALIZED) {
                state_ = CALIBRATED;
            }
            calibrated_ = true;
            params_ = action.params;
        }
    }

    void start() {
      state_ = STARTING;
      worker_ = std::thread([&]() {
        startColorStream();
        startDepthStream();
        startIrStream();
        enableMirroring();
        enableDepthToColorRegistration();

        if (!device_.isFile()) {
          // TODO: record .oni file
        }

        fps_counter_.start(serial_);
        worker_canceled_ = false;
        state_ = PLAYING;
        while (!worker_canceled_) {
            update();
            fps_counter_.passFrame();
        }
      });
    }

    void stop() {
        state_ = STOPPING;
        worker_canceled_ = true;
        stopRecording();
        if (worker_.joinable()) {
            worker_.join();
        }
        if (color_stream_.isValid()) {
            color_stream_.stop();
            color_stream_.destroy();
        }
        if (depth_stream_.isValid()) {
            depth_stream_.stop();
            depth_stream_.destroy();
        }
        if (ir_stream_.isValid()) {
            ir_stream_.stop();
            ir_stream_.destroy();
        }
        fps_counter_.stop();
        state_ = calibrated_ ? CALIBRATED : INITIALIZED;
    }

    void record(std::string dir) {
        std::ostringstream ss;
        auto now = boost::posix_time::microsec_clock::universal_time();
        ss << dir << "/" << boost::posix_time::to_iso_string(now) << "_" << serial_ << ".oni";
        checkStatus(recorder_.create(ss.str().c_str()), "Creating recorder failed.");
        checkStatus(recorder_.attach(color_stream_, TRUE), "Attaching color stream to recorder failed.");
        checkStatus(recorder_.attach(depth_stream_, FALSE), "Attaching depth stream to recorder failed.");
        checkStatus(recorder_.start(), "Recording failed.");
        recording_ = true;
        state_ = RECORDING;
    }

    void stopRecording() {
        if (recorder_.isValid()) {
            recorder_.stop();
            recorder_.destroy();
            if (state_ == RECORDING) {
                state_ = PLAYING;
            }
            recording_ = false;
        }
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

    void updateFps(const FpsCounter::Event& event) {
        if (event.key == serial_) {
            fps_ = event.fps;
        }
    }

    void checkStatus(openni::Status status, std::string msg) {
        if (status != openni::STATUS_OK) { 
            throw std::runtime_error(msg);
        }
    }

    void startColorStream() {
      if (device_.hasSensor(openni::SENSOR_COLOR)) {
        checkStatus(color_stream_.create(device_, openni::SENSOR_COLOR), "Color stream failed to create.");
        const openni::Array<openni::VideoMode> *supported_video_modes
            = &(color_stream_.getSensorInfo().getSupportedVideoModes());
        int num_of_video_modes = supported_video_modes->getSize();
        if (num_of_video_modes == 0) {
          throw std::runtime_error("VideoMode failed.");
        }
        checkStatus(color_stream_.setVideoMode((* supported_video_modes)[1]), "Set video mode to color stream failed");
        checkStatus(color_stream_.start(), "Color stream failed to start.");
      }
    }

    void startDepthStream() {
      if (device_.hasSensor(openni::SENSOR_DEPTH)) {
        checkStatus(depth_stream_.create(device_, openni::SENSOR_DEPTH), "Depth stream failed to create.");
        const openni::Array<openni::VideoMode> *supported_video_modes
            = &(depth_stream_.getSensorInfo().getSupportedVideoModes());
        int num_of_video_modes = supported_video_modes->getSize();
        if (num_of_video_modes == 0) {
          throw std::runtime_error("VideoMode failed.");
        }
        checkStatus(depth_stream_.setVideoMode((* supported_video_modes)[0]), "Set video mode to depth stream failed");
        checkStatus(depth_stream_.start(), "Depth stream failed to start.");
      }
    }

    void startIrStream() {
      if (device_.hasSensor(openni::SENSOR_IR) && name_ == "freenect2") {
        checkStatus(depth_stream_.create(device_, openni::SENSOR_IR), "IR stream failed to create.");
        const openni::Array<openni::VideoMode> *supported_video_modes
            = &(ir_stream_.getSensorInfo().getSupportedVideoModes());
        int num_of_video_modes = supported_video_modes->getSize();
        if (num_of_video_modes == 0) {
          throw std::runtime_error("VideoMode failed.");
        }
        checkStatus(depth_stream_.start(), "Color stream failed to start.");
      }
    }

    void enableMirroring() {
      if (!device_.isFile()) {
        if (color_stream_.isValid()) {
          checkStatus(color_stream_.setMirroringEnabled(true), "Color sensor mirroring failed.");
        }
        if (depth_stream_.isValid()) {
          checkStatus(depth_stream_.setMirroringEnabled(true), "Depth sensor mirroring failed.");
        }
        if (ir_stream_.isValid()) {
          checkStatus(ir_stream_.setMirroringEnabled(true), "IR sensor mirroring failed.");
        }
      }
    }

    void enableDepthToColorRegistration() {
      if (color_stream_.isValid() && depth_stream_.isValid()) {
        // checkStatus(device_.setDepthColorSyncEnabled(true), "Depth-Color sync failed.");

        device_.setDepthColorSyncEnabled(true);
        checkStatus(device_.setImageRegistrationMode(
              openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR), "Color to depth registration failed.");
      }
    }

    void update() {
      openni::VideoFrameRef color_frame;
      openni::VideoFrameRef depth_frame;
      openni::VideoFrameRef ir_frame;

      if (color_stream_.isValid()) {
        checkStatus(color_stream_.readFrame(&color_frame), "Failed to read color frame.");
        updateColorImage(color_frame);
      }
      if (depth_stream_.isValid()) {
        checkStatus(depth_stream_.readFrame(&depth_frame), "Failed to read depth frame.");
        updateRawDepthImage(depth_frame);
//        updateDepthImage(depth_frame);
      }
//      if (ir_stream_.isValid()) {
//        checkStatus(ir_stream_.readFrame(&ir_frame), "Failed to read ir frame.");
//        updateIrImage(ir_frame);
//      }

      auto now = std::chrono::system_clock::now();
      updatePointCloud(now);
    }

    void updateColorImage(const openni::VideoFrameRef &color_frame) {
      auto color_image = cv::Mat(color_frame.getHeight(), color_frame.getWidth(),
                                 CV_8UC3, (unsigned char*) color_frame.getData());
      cv::cvtColor(color_image, color_image_, CV_RGB2BGR);
    }

    void updateRawDepthImage(const openni::VideoFrameRef &depth_frame) {
      raw_depth_image_ = cv::Mat(depth_frame.getHeight(), depth_frame.getWidth(),
                                 CV_16UC1,
                                 (unsigned short *) depth_frame.getData());
    }

    void updateDepthImage(const openni::VideoFrameRef &depth_frame) {
      cv::Mat depth_image;
      cv::Mat(depth_frame.getHeight(), depth_frame.getWidth(), CV_16UC1,
            (unsigned short *) depth_frame.getData()).copyTo(depth_image);
      depth_image.convertTo(depth_image, CV_8U, 255.0 / 10000);
      // FIXME: hardcoding?
      cv::Rect roi(0, 0, 512, 424);
      depth_image_ = depth_image(roi);
    }

    void updateIrImage(const openni::VideoFrameRef &ir_frame) {
      cv::Mat ir_image;
      cv::Mat(ir_frame.getHeight(), ir_frame.getWidth(), CV_16UC1,
            (unsigned short *) ir_frame.getData()).copyTo(ir_image);
      ir_image.convertTo(ir_image, CV_8U, 255.0 / 10000);
      cv::cvtColor(ir_image, ir_image, CV_GRAY2RGB);
      ir_image_ = ir_image;
    }

    void updatePointCloud(std::chrono::system_clock::time_point timestamp) {
      int width   = raw_depth_image_.size().width;
      int height  = raw_depth_image_.size().height;

      unsigned char* color = (unsigned char*) color_image_.data;
      unsigned short* depth = (unsigned short*) raw_depth_image_.data;

      VerticesPtr vertices(new Vertices);

      for (int i = 0; i < width * height; i++) {
        if (depth[i] != 0 && (color[i*3] != 0 || color[i*3+1] != 0 || color[i*3+2] != 0)) {
          vertices->emplace_back((Vertex){
            {
              static_cast<float>(i % width),
              static_cast<float>(i / width),
              static_cast<float>(depth[i])
            },
            {
              static_cast<uint8_t>(color[i * 3 + 2]),
              static_cast<uint8_t>(color[i * 3 + 1]),
              static_cast<uint8_t>(color[i * 3])
            }
          });
        }
      }

      Signal<Clouds::UpdateVerticesAction>::emit({name_, vertices, timestamp});
    }
};

}

#endif /* SensorDevice_hpp */
