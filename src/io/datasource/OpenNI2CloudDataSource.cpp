//
// Created by Masayuki IZUMI on 10/7/16.
//

#include "Clouds.h"
#include "Signal.h"
#include "io/datasource/OpenNI2CloudDataSource.h"
#include "io/datasource/SkeletonTracker.h"
#include "util/util.h"

namespace io {
namespace datasource {

OpenNI2CloudDataSource::OpenNI2CloudDataSource(const std::string name, const std::string uri)
  : CloudDataSource (name)
  , uri_            (uri)
  , color_stream_   (new openni::VideoStream)
  , depth_stream_   (new openni::VideoStream)
  , ir_stream_      (new openni::VideoStream)
  , user_count_     (0)
  , updated_color_image_(false)
  , updated_depth_image_(false)
  , updated_ir_image_   (false)
#ifdef USE_NITE2
  , updated_user_image_ (false)
#endif
{
}

void OpenNI2CloudDataSource::onNewFrame(openni::VideoStream &stream) {
  openni::VideoFrameRef frame;
  util::checkStatus(stream.readFrame(&frame), "Failed to read a video frame.");
  switch (stream.getSensorInfo().getSensorType()) {
    case openni::SENSOR_COLOR:
      updateColorImage(frame);
      updated_color_image_ = true;
      break;
    case openni::SENSOR_DEPTH:
#ifdef USE_NITE2
#else
      updateRawDepthImage(frame);
//      updateDepthImage(frame);
      updated_depth_image_ = true;
#endif
      break;
    case openni::SENSOR_IR:
      updateIrImage(frame);
      updated_ir_image_ = true;
      break;
    default:
      throw std::runtime_error("Received an unknown video frame.");
  }
  updated_cond_.notify_all();
}

#ifdef USE_NITE2
void OpenNI2CloudDataSource::onNewFrame(nite::UserTracker &tracker) {
  nite::UserTrackerFrameRef frame;
  util::checkStatus(tracker.readFrame(&frame), "Failed to read an user frame.");
  auto depth_frame = frame.getDepthFrame();
  skeletons_ = SkeletonTracker::getSkeletons(tracker, frame);
  updateRawDepthImage(depth_frame);
//  updateDepthImage(depth_frame);
  updateUserImage(frame);
  updated_user_image_ = true;
  user_count_ = frame.getUsers().getSize();
  updated_cond_.notify_all();
}
#endif

void OpenNI2CloudDataSource::onStart() {
  util::checkStatus(device_.open(uri_.c_str()), "openni::Device::open() failed.");

  startColorStream();
  startDepthStream();
//  startIrStream();
  enableMirroring();
  enableDepthToColorRegistration();
#ifdef USE_NITE2
  startUserTracker();
#endif
}

void OpenNI2CloudDataSource::onStop() {
  if (color_stream_->isValid()) {
    color_stream_->removeNewFrameListener(this);
    color_stream_->stop();
    color_stream_->destroy();
  }
#ifdef USE_NITE2
  if (user_tracker_.isValid()) {
    user_tracker_.removeNewFrameListener(this);
    user_tracker_.destroy();
  }
#endif
  if (depth_stream_->isValid()) {
#ifdef USE_NITE2
#else
    depth_stream_->removeNewFrameListener(this);
#endif
    depth_stream_->stop();
    depth_stream_->destroy();
  }
//  if (ir_stream_->isValid()) {
//    ir_stream_->removeNewFrameListener(this);
//    ir_stream_->stop();
//    ir_stream_->destroy();
//  }
  if (device_.isValid()) {
    device_.close();
  }
}

void OpenNI2CloudDataSource::update() {
  {
    std::unique_lock<std::mutex> lk(mutex_updated_);
    updated_cond_.wait(lk, [this] {
#ifdef USE_NITE2
      return updated_color_image_ && updated_user_image_;
#else
      return updated_color_image_ && updated_depth_image_;
#endif
    });

#ifdef USE_NITE2
    updated_color_image_ = updated_user_image_ = false;
#else
    updated_color_image_ = updated_depth_image_ = false;
#endif
  }

  updatePointCloud(util::now());
}

std::shared_ptr<openni::VideoStream> OpenNI2CloudDataSource::getColorVideoStream() {
  return color_stream_;
}

std::shared_ptr<openni::VideoStream> OpenNI2CloudDataSource::getDepthVideoStream() {
  return depth_stream_;
}

void OpenNI2CloudDataSource::startColorStream() {
  if (device_.hasSensor(openni::SENSOR_COLOR)) {
    util::checkStatus(color_stream_->create(device_, openni::SENSOR_COLOR), "Color stream failed to create.");
    const openni::Array<openni::VideoMode> *supported_video_modes
      = &(color_stream_->getSensorInfo().getSupportedVideoModes());
    int num_of_video_modes = supported_video_modes->getSize();
    if (num_of_video_modes == 0) {
      throw std::runtime_error("VideoMode failed.");
    }
    for (int i = 0; i < num_of_video_modes; i++) {
      openni::VideoMode vm = (*supported_video_modes)[i];
      printf("%c. %dx%d at %dfps with %d format \r\n", '0' + i,
             vm.getResolutionX(), vm.getResolutionY(), vm.getFps(),
             vm.getPixelFormat());
    }
    util::checkStatus(color_stream_->setVideoMode((* supported_video_modes)[1]), "Set video mode to color stream failed");
    util::checkStatus(color_stream_->start(), "Color stream failed to start.");
    util::checkStatus(color_stream_->addNewFrameListener(this), "Failed to add the new color frame listener.");
  }
}

void OpenNI2CloudDataSource::startDepthStream() {
  if (device_.hasSensor(openni::SENSOR_DEPTH)) {
    util::checkStatus(depth_stream_->create(device_, openni::SENSOR_DEPTH), "Depth stream failed to create.");
    const openni::Array<openni::VideoMode> *supported_video_modes
      = &(depth_stream_->getSensorInfo().getSupportedVideoModes());
    int num_of_video_modes = supported_video_modes->getSize();
    if (num_of_video_modes == 0) {
      throw std::runtime_error("VideoMode failed.");
    }
    for (int i = 0; i < num_of_video_modes; i++) {
      openni::VideoMode vm = (*supported_video_modes)[i];
      printf("%c. %dx%d at %dfps with %d format \r\n", '0' + i,
             vm.getResolutionX(), vm.getResolutionY(), vm.getFps(),
             vm.getPixelFormat());
    }
    util::checkStatus(depth_stream_->setVideoMode((* supported_video_modes)[1]), "Set video mode to depth stream failed");
    util::checkStatus(depth_stream_->start(), "Depth stream failed to start.");
#ifdef USE_NITE2
#else
    util::checkStatus(depth_stream_->addNewFrameListener(this), "Failed to add the new depth frame listener.");
#endif
  }
}

void OpenNI2CloudDataSource::startIrStream() {
  if (device_.hasSensor(openni::SENSOR_IR)) {
    util::checkStatus(ir_stream_->create(device_, openni::SENSOR_IR), "IR stream failed to create.");
    const openni::Array<openni::VideoMode> *supported_video_modes
      = &(ir_stream_->getSensorInfo().getSupportedVideoModes());
    int num_of_video_modes = supported_video_modes->getSize();
    if (num_of_video_modes == 0) {
      throw std::runtime_error("VideoMode failed.");
    }
    util::checkStatus(ir_stream_->start(), "Ir stream failed to start.");
    util::checkStatus(ir_stream_->addNewFrameListener(this), "Failed to add the new ir frame listener.");
  }
}

#ifdef USE_NITE2
void OpenNI2CloudDataSource::startUserTracker() {
  util::checkStatus(user_tracker_.create(&device_), "Failed to create user tracker.");
  user_tracker_.addNewFrameListener(this);
}
#endif

void OpenNI2CloudDataSource::enableMirroring() {
  if (!device_.isFile()) {
    if (color_stream_->isValid()) {
      util::checkStatus(color_stream_->setMirroringEnabled(true), "Color sensor mirroring failed.");
    }
    if (depth_stream_->isValid()) {
      util::checkStatus(depth_stream_->setMirroringEnabled(true), "Depth sensor mirroring failed.");
    }
    if (ir_stream_->isValid()) {
      util::checkStatus(ir_stream_->setMirroringEnabled(true), "IR sensor mirroring failed.");
    }
  }
}

void OpenNI2CloudDataSource::enableDepthToColorRegistration() {
  if (color_stream_->isValid() && depth_stream_->isValid()) {
    // util::checkStatus(device_.setDepthColorSyncEnabled(true), "Depth-Color sync failed.");
    device_.setDepthColorSyncEnabled(true);

    util::checkStatus(device_.setImageRegistrationMode(
      openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR), "Color to depth registration failed.");
  }
}

void OpenNI2CloudDataSource::updateColorImage(const openni::VideoFrameRef &color_frame) {
  auto color_image = cv::Mat(color_frame.getHeight(), color_frame.getWidth(),
                             CV_8UC3, (unsigned char*) color_frame.getData());
  std::lock_guard<std::mutex> lg(mutex_color_image_);
  cv::cvtColor(color_image, color_image_, CV_RGB2BGR);
}

void OpenNI2CloudDataSource::updateRawDepthImage(const openni::VideoFrameRef &depth_frame) {
  cv::Mat image;
  cv::Mat(depth_frame.getHeight(), depth_frame.getWidth(),
          CV_16UC1,
          (unsigned short *) depth_frame.getData()
  ).copyTo(image);
  cv::Rect roi(0, 0, 512, 424);
  std::lock_guard<std::mutex> lg(mutex_raw_depth_image_);
  raw_depth_image_ = image(roi);
}

void OpenNI2CloudDataSource::updateDepthImage(const openni::VideoFrameRef &depth_frame) {
  cv::Mat depth_image;
  cv::Mat(depth_frame.getHeight(), depth_frame.getWidth(), CV_16UC1,
          (unsigned short *) depth_frame.getData()).copyTo(depth_image);
  depth_image.convertTo(depth_image, CV_8U, 255.0 / 10000);
  cv::Rect roi(0, 0, 512, 424);
  std::lock_guard<std::mutex> lg(mutex_depth_image_);
  depth_image_ = depth_image(roi);
}

void OpenNI2CloudDataSource::updateIrImage(const openni::VideoFrameRef &ir_frame) {
  cv::Mat ir_image;
  cv::Mat(ir_frame.getHeight(), ir_frame.getWidth(), CV_16UC1,
          (unsigned short *) ir_frame.getData()).copyTo(ir_image);
  ir_image.convertTo(ir_image, CV_8U, 255.0 / 10000);
  cv::cvtColor(ir_image, ir_image, CV_GRAY2RGB);
  std::lock_guard<std::mutex> lg(mutex_ir_image_);
  ir_image_ = ir_image;
}

#ifdef USE_NITE2
void OpenNI2CloudDataSource::updateUserImage(const nite::UserTrackerFrameRef &user_frame) {
  std::lock_guard<std::mutex> lg(mutex_user_image_);
  user_ids_ = user_frame.getUserMap().getPixels();
}
#endif

void OpenNI2CloudDataSource::updatePointCloud(std::chrono::system_clock::time_point timestamp) {
  std::lock_guard<std::mutex> lg_raw_depth(mutex_raw_depth_image_);

  int width   = raw_depth_image_.size().width;
  int height  = raw_depth_image_.size().height;

  VerticesPtr vertices(new Vertices);
  vertices->reserve((size_t) (width * height));

  std::lock_guard<std::mutex> lg_color(mutex_color_image_);
#ifdef USE_NITE2
  std::lock_guard<std::mutex> lg_user(mutex_user_image_);
#endif

  for (auto &p1 : *skeletons_) {
    for (auto &p2 : p1.second) {
      int x = (int) p2.second.x, y = (int) p2.second.y;
      p2.second.z = raw_depth_image_.ptr(y)[x];
    }
  }

  for (int y = 0; y < height; y++) {
    unsigned short *depth = (unsigned short *) raw_depth_image_.ptr(y);
    unsigned char *color = (unsigned char *) color_image_.ptr(y);

    for (int x = 0; x < width; x++) {
      if (depth[x] != 0 && (color[x*3] != 0 || color[x*3+1] != 0 || color[x*3+2] != 0)) {
        vertices->emplace_back((Vertex) {
          {
            static_cast<float>(x),
            static_cast<float>(y),
            static_cast<float>(depth[x])
          },
          {
            static_cast<uint8_t>(color[x * 3 + 2]),
            static_cast<uint8_t>(color[x * 3 + 1]),
            static_cast<uint8_t>(color[x * 3 + 0])
          }
#ifdef USE_NITE2
        , user_ids_[y * 640 + x]
#endif
        });
      }
    }
  }

  Signal<Clouds::UpdateVerticesAction>::emit({name(), vertices, timestamp, user_count_});
  Signal<Clouds::UpdateSkeletonsAction>::emit({name(), skeletons_, timestamp});
}

}
}
