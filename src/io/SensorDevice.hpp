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

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/utility.hpp>
#include <OpenNI.h>

#include "FpsCounter.h"
#include "Signal.h"
#include "model/CloudsManager.h"
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
      : worker_canceled_(true)
      , calibrated_(false)
      , recording_(false)
      , fps_(0.0f)
      , state_(NOT_INITIALIZED)
    {
    }

    ~SensorDevice() {
        stop();
        device_.close();
    }

    void initialize(const char *uri = openni::ANY_DEVICE) {
        uri_ = uri;

        Signal<CalibrationParams>::connect(this, &SensorDevice::setCalibrationParams);
        Signal<FpsCounter::Event>::connect(this, &SensorDevice::updateFps);

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

    void setCalibrationParams(const CalibrationParams& params) {
        if (params.serial == serial_) {
            if (state_ == INITIALIZED) {
                state_ = CALIBRATED;
            }
            calibrated_ = true;
            params_ = params;
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
        state_ = calibrated_ ? CALIBRATED : INITIALIZED;
    }

    void record(boost::filesystem::path dir) {
        std::ostringstream ss;
        auto now = boost::posix_time::microsec_clock::universal_time();
        ss << boost::posix_time::to_iso_string(now) << "_" << serial_ << ".oni";
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

    cv::UMat color_image_;
    cv::UMat raw_depth_image_;
    cv::UMat depth_image_;
    cv::UMat ir_image_;

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
        checkStatus(depth_stream_.start(), "Color stream failed to start.");
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
        color_image_ = updateColorImage(color_frame);
      }
      if (depth_stream_.isValid()) {
        checkStatus(depth_stream_.readFrame(&depth_frame), "Failed to read depth frame.");
        raw_depth_image_ = updateRawDepthImage(depth_frame);
        depth_image_ = updateDepthImage(depth_frame);
      }
      if (ir_stream_.isValid()) {
        checkStatus(ir_stream_.readFrame(&ir_frame), "Failed to read ir frame.");
        ir_image_ = updateIrImage(ir_frame);
      }

      updatePointCloud();
    }

    static cv::UMat updateColorImage(const openni::VideoFrameRef &color_frame) {
      cv::UMat color_image;
      cv::Mat(color_frame.getHeight(), color_frame.getWidth(), CV_8UC3,
              (unsigned char *) color_frame.getData()).copyTo(color_image);
      cv::cvtColor(color_image, color_image, CV_RGB2BGR);
      return color_image;
    }

    static cv::UMat updateRawDepthImage(const openni::VideoFrameRef &depth_frame) {
      cv::UMat raw_depth_image;
      cv::Mat(depth_frame.getHeight(), depth_frame.getWidth(), CV_16UC1,
            (unsigned short *) depth_frame.getData()).copyTo(raw_depth_image);
      // FIXME: hardcoding?
      cv::Rect roi(0, 0, 512, 424);
      return raw_depth_image(roi);
    }

    static cv::UMat updateDepthImage(const openni::VideoFrameRef &depth_frame) {
      cv::UMat depth_image;
      cv::Mat(depth_frame.getHeight(), depth_frame.getWidth(), CV_16UC1,
            (unsigned short *) depth_frame.getData()).copyTo(depth_image);
      depth_image.convertTo(depth_image, CV_8U, 255.0 / 10000);
      // FIXME: hardcoding?
      cv::Rect roi(0, 0, 512, 424);
      return depth_image(roi);
    }

    static cv::UMat updateIrImage(const openni::VideoFrameRef &ir_frame) {
      cv::UMat ir_image;
      cv::Mat(ir_frame.getHeight(), ir_frame.getWidth(), CV_16UC1,
            (unsigned short *) ir_frame.getData()).copyTo(ir_image);
      ir_image.convertTo(ir_image, CV_8U, 255.0 / 10000);
      cv::cvtColor(ir_image, ir_image, CV_GRAY2RGB);
      return ir_image;
     }

    void updatePointCloud() {
      int width   = color_image_.size().width;
      int height  = color_image_.size().height;
      // FIXME: hardcoding
      int dwidth  = 640;

      auto cv_depth_image = raw_depth_image_.getMat(cv::ACCESS_READ);
      auto cv_color_image = color_image_.getMat(cv::ACCESS_READ);

      const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
      cloud->clear();
      cloud->reserve(width * height);

      for (int y = 0; y < height; y++) {
        unsigned short *depth = (unsigned short *) cv_depth_image.ptr(y);
        cv::Vec3b *color = (cv::Vec3b *) cv_color_image.ptr(y);

        for (int x = 0; x < width; x++) {
          if (depth[x] != 0 && (color[x][0] != 0 || color[x][1] != 0 || color[x][2] != 0)) {
            float xw, yw, zw;
            zw = depth[x];
            zw /= 1000;
            xw = (x - params_.cx) * zw / params_.fx;
            yw = (y - params_.cy) * zw / params_.fy;
            pcl::PointXYZRGBA point;
            point.x = xw;
            point.y = yw;
            point.z = zw;
            point.b = color[x][0];
            point.g = color[x][1];
            point.r = color[x][2];
            cloud->push_back(point);
          }
        }
      }

      pcl::transformPointCloud(*cloud, *cloud, params_.calib_matrix);
      Signal<models::CloudEvent>::emit({name_, cloud});
    }
};

}

#endif /* SensorDevice_hpp */
