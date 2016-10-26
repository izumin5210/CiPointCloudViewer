//
// Created by Masayuki IZUMI on 10/26/16.
//

#include "Clouds.h"
#include "Signal.h"
#include "io/datasource/OpenNI2GrabberCloudDataSource.h"

namespace io {
namespace datasource {

OpenNI2GrabberCloudDataSource::OpenNI2GrabberCloudDataSource(
  const std::string name,
  const std::string uri
)
  : CloudDataSource(name)
  , uri_(uri)
  , updated_(false)
{}

void OpenNI2GrabberCloudDataSource::onStart() {
  grabber_ = std::make_unique<pcl::io::OpenNI2Grabber>(getDeviceIdForGrabber());
  boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> cb =
    boost::bind(&OpenNI2GrabberCloudDataSource::onPointCloudReceived, this, _1);
  conn_ = grabber_->registerCallback(cb);
  grabber_->start();
}

void OpenNI2GrabberCloudDataSource::onStop() {
  if (grabber_->isRunning()) {
    grabber_->stop();
  }
  if (conn_.connected()) {
    conn_.disconnect();
  }
}

void OpenNI2GrabberCloudDataSource::update() {
  std::unique_lock<std::mutex> lk(mutex_);
  cond_updated_.wait(lk, [this] { return updated_; });
  updated_ = false;
}

std::shared_ptr<openni::VideoStream> OpenNI2GrabberCloudDataSource::getColorVideoStream() {
  // TODO: Not yet implemented.
  return nullptr;
}

std::shared_ptr<openni::VideoStream> OpenNI2GrabberCloudDataSource::getDepthVideoStream() {
  // TODO: Not yet implemented.
  return nullptr;
}

std::string OpenNI2GrabberCloudDataSource::getDeviceIdForGrabber() {
  // NOTE: https://gist.github.com/izumin5210/3f610c6f69ab9b20b7059b948490d229
  auto device_manager = pcl::io::openni2::OpenNI2DeviceManager::getInstance();
  auto uris = device_manager->getConnectedDeviceURIs();
  std::stringstream ss;
  ss << "#";
  for (size_t i = 0; i < uris->size(); i++) {
    if (uris->at(i) == uri_) {
      ss << i + 1;
    }
  }
  return ss.str();
}

void OpenNI2GrabberCloudDataSource::onPointCloudReceived(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
  std::lock_guard<std::mutex> lk(mutex_);
  Signal<Clouds::UpdatePointsAction>::emit({name(), (*cloud).makeShared()});
  updated_ = true;
  cond_updated_.notify_one();
}

}
}