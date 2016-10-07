//
// Created by Masayuki IZUMI on 10/7/16.
//

#include "Clouds.h"
#include "Signal.h"
#include "io/datasource/OpenNI2CloudDataSource.h"

namespace io {
namespace datasource {

OpenNI2CloudDataSource::OpenNI2CloudDataSource(const std::string name, const std::shared_ptr<openni::Device> &device)
  : CloudDataSource(name)
  , device_(device)
  , color_stream_(new openni::VideoStream)
  , depth_stream_(new openni::VideoStream)
  , ir_stream_(new openni::VideoStream)
{
}

void OpenNI2CloudDataSource::onStart() {
  startColorStream();
  startDepthStream();
  startIrStream();
  enableMirroring();
  enableDepthToColorRegistration();
}

void OpenNI2CloudDataSource::onStop() {
  if (color_stream_->isValid()) {
    color_stream_->stop();
    color_stream_->destroy();
  }
  if (depth_stream_->isValid()) {
    depth_stream_->stop();
    depth_stream_->destroy();
  }
  if (ir_stream_->isValid()) {
    ir_stream_->stop();
    ir_stream_->destroy();
  }
}

void OpenNI2CloudDataSource::update() {
  openni::VideoFrameRef color_frame;
  openni::VideoFrameRef depth_frame;
  openni::VideoFrameRef ir_frame;

  if (color_stream_->isValid()) {
    checkStatus(color_stream_->readFrame(&color_frame), "Failed to read color frame.");
    updateColorImage(color_frame);
  }
  if (depth_stream_->isValid()) {
    checkStatus(depth_stream_->readFrame(&depth_frame), "Failed to read depth frame.");
    updateRawDepthImage(depth_frame);
//        updateDepthImage(depth_frame);
  }
//      if (ir_stream_->isValid()) {
//        checkStatus(ir_stream_->readFrame(&ir_frame), "Failed to read ir frame.");
//        updateIrImage(ir_frame);
//      }

  auto now = std::chrono::system_clock::now();
  updatePointCloud(now);
}

std::shared_ptr<openni::VideoStream> OpenNI2CloudDataSource::getColorVideoStream() {
  return color_stream_;
}

std::shared_ptr<openni::VideoStream> OpenNI2CloudDataSource::getDepthVideoStream() {
  return depth_stream_;
}

void OpenNI2CloudDataSource::startColorStream() {
  if (device_->hasSensor(openni::SENSOR_COLOR)) {
    checkStatus(color_stream_->create(*device_, openni::SENSOR_COLOR), "Color stream failed to create.");
    const openni::Array<openni::VideoMode> *supported_video_modes
      = &(color_stream_->getSensorInfo().getSupportedVideoModes());
    int num_of_video_modes = supported_video_modes->getSize();
    if (num_of_video_modes == 0) {
      throw std::runtime_error("VideoMode failed.");
    }
    checkStatus(color_stream_->setVideoMode((* supported_video_modes)[1]), "Set video mode to color stream failed");
    checkStatus(color_stream_->start(), "Color stream failed to start.");
  }
}

void OpenNI2CloudDataSource::startDepthStream() {
  if (device_->hasSensor(openni::SENSOR_DEPTH)) {
    checkStatus(depth_stream_->create(*device_, openni::SENSOR_DEPTH), "Depth stream failed to create.");
    const openni::Array<openni::VideoMode> *supported_video_modes
      = &(depth_stream_->getSensorInfo().getSupportedVideoModes());
    int num_of_video_modes = supported_video_modes->getSize();
    if (num_of_video_modes == 0) {
      throw std::runtime_error("VideoMode failed.");
    }
    checkStatus(depth_stream_->setVideoMode((* supported_video_modes)[0]), "Set video mode to depth stream failed");
    checkStatus(depth_stream_->start(), "Depth stream failed to start.");
  }
}

void OpenNI2CloudDataSource::startIrStream() {
  if (device_->hasSensor(openni::SENSOR_IR)) {
    checkStatus(ir_stream_->create(*device_, openni::SENSOR_IR), "IR stream failed to create.");
    const openni::Array<openni::VideoMode> *supported_video_modes
      = &(ir_stream_->getSensorInfo().getSupportedVideoModes());
    int num_of_video_modes = supported_video_modes->getSize();
    if (num_of_video_modes == 0) {
      throw std::runtime_error("VideoMode failed.");
    }
    checkStatus(ir_stream_->start(), "Ir stream failed to start.");
  }
}

void OpenNI2CloudDataSource::enableMirroring() {
  if (!device_->isFile()) {
    if (color_stream_->isValid()) {
      checkStatus(color_stream_->setMirroringEnabled(true), "Color sensor mirroring failed.");
    }
    if (depth_stream_->isValid()) {
      checkStatus(depth_stream_->setMirroringEnabled(true), "Depth sensor mirroring failed.");
    }
    if (ir_stream_->isValid()) {
      checkStatus(ir_stream_->setMirroringEnabled(true), "IR sensor mirroring failed.");
    }
  }
}

void OpenNI2CloudDataSource::enableDepthToColorRegistration() {
  if (color_stream_->isValid() && depth_stream_->isValid()) {
    // checkStatus(device_->setDepthColorSyncEnabled(true), "Depth-Color sync failed.");
    device_->setDepthColorSyncEnabled(true);

    checkStatus(device_->setImageRegistrationMode(
      openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR), "Color to depth registration failed.");
  }
}

void OpenNI2CloudDataSource::updateColorImage(const openni::VideoFrameRef &color_frame) {
  auto color_image = cv::Mat(color_frame.getHeight(), color_frame.getWidth(),
                             CV_8UC3, (unsigned char*) color_frame.getData());
  cv::cvtColor(color_image, color_image_, CV_RGB2BGR);
}

void OpenNI2CloudDataSource::updateRawDepthImage(const openni::VideoFrameRef &depth_frame) {
  raw_depth_image_ = cv::Mat(depth_frame.getHeight(), depth_frame.getWidth(),
                             CV_16UC1,
                             (unsigned short *) depth_frame.getData());
}

void OpenNI2CloudDataSource::updateDepthImage(const openni::VideoFrameRef &depth_frame) {
  cv::Mat depth_image;
  cv::Mat(depth_frame.getHeight(), depth_frame.getWidth(), CV_16UC1,
          (unsigned short *) depth_frame.getData()).copyTo(depth_image);
  depth_image.convertTo(depth_image, CV_8U, 255.0 / 10000);
  // FIXME: hardcoding?
  cv::Rect roi(0, 0, 512, 424);
  depth_image_ = depth_image(roi);
}

void OpenNI2CloudDataSource::updateIrImage(const openni::VideoFrameRef &ir_frame) {
  cv::Mat ir_image;
  cv::Mat(ir_frame.getHeight(), ir_frame.getWidth(), CV_16UC1,
          (unsigned short *) ir_frame.getData()).copyTo(ir_image);
  ir_image.convertTo(ir_image, CV_8U, 255.0 / 10000);
  cv::cvtColor(ir_image, ir_image, CV_GRAY2RGB);
  ir_image_ = ir_image;
}

void OpenNI2CloudDataSource::updatePointCloud(std::chrono::system_clock::time_point timestamp) {
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

  Signal<Clouds::UpdateVerticesAction>::emit({name(), vertices, timestamp});
}

}
}
