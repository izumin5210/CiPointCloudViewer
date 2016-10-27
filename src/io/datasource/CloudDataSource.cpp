//
// Created by Masayuki IZUMI on 10/7/16.
//

#include <thread>
#include <boost/date_time.hpp>

#include "Signal.h"
#include "io/datasource/CloudDataSource.h"
#include "util/util.h"

namespace io {
namespace datasource {

CloudDataSource::CloudDataSource(const std::string name)
  : name_(name)
  , state_(STOPPED)
  , fps_(0.0)
{
  auto cb = std::bind(&CloudDataSource::updateFps, this, std::placeholders::_1);
  Signal<FpsCounter::Event>::connect(cb);
}

CloudDataSource::~CloudDataSource() {
  stopRecording();
  stopFetching();
  if (stop_worker_.joinable()) {
    stop_worker_.join();
  }
}

void CloudDataSource::startFetching() {
  if (stop_worker_.joinable()) {
    stop_worker_.join();
  }
  worker_ = std::thread([&]() {
    state_ = STARTING;
    onStart();

    fps_counter_.start(name_);

    state_ = RUNNING;
    while (isRunning()) {
      update();
      fps_counter_.passFrame();
    }
  });
}

void CloudDataSource::stopFetching() {
  state_ = STOPPING;
  stop_worker_ = std::thread([&]() {
    fps_counter_.stop();
    worker_.join();
    onStop();
    state_ = STOPPED;
  });
}

void CloudDataSource::startRecording(std::string dir) {
  std::ostringstream ss;
  auto now = boost::posix_time::microsec_clock::universal_time();
  ss << dir << "/" << boost::posix_time::to_iso_string(now) << "_" << name() << ".oni";
  util::checkStatus(recorder_.create(ss.str().c_str()), "Creating recorder failed.");
  util::checkStatus(recorder_.attach(*getColorVideoStream(), TRUE), "Attaching color stream to recorder failed.");
  util::checkStatus(recorder_.attach(*getDepthVideoStream(), FALSE), "Attaching depth stream to recorder failed.");
  util::checkStatus(recorder_.start(), "Recording failed.");
  state_ = RECORDING;
}

void CloudDataSource::stopRecording() {
  if (recorder_.isValid()) {
    recorder_.stop();
    recorder_.destroy();
    if (state_ == RECORDING) {
      state_ = RUNNING;
    }
  }
}

void CloudDataSource::updateFps(const FpsCounter::Event& event) {
  if (event.key == name_) {
    fps_ = event.fps;
  }
}

}
}
