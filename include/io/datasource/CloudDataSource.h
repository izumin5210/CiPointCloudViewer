//
// Created by Masayuki IZUMI on 10/7/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_CLOUDDATASOURCE_H
#define CIPOINTCLOUDVIEWERAPP_CLOUDDATASOURCE_H

#include <map>
#include <OpenNI.h>
#ifdef USE_NITE2
#include <NiTE.h>
#endif

#include "FpsCounter.h"

namespace io {
namespace datasource {

class CloudDataSource {
public:
  enum State {
    STOPPED,
    STARTING,
    RUNNING,
    RECORDING,
    STOPPING
  };

  CloudDataSource(const std::string name);
  virtual ~CloudDataSource();
  void startFetching();
  void stopFetching();
  void startRecording(std::string dir);
  void stopRecording();

  State state() const {
    return state_;
  }

  float fps() const {
    return fps_;
  }

  bool isRunning() const {
    return state_ == RUNNING || state_ == RECORDING;
  }

  bool isRecording() const {
    return state_ == RECORDING;
  }

  std::string stateString() {
    return kStateStrings[state()];
  }


protected:
  virtual void onStart() = 0;
  virtual void onStop() = 0;
  virtual void update() = 0;
  virtual std::shared_ptr<openni::VideoStream> getColorVideoStream() = 0;
  virtual std::shared_ptr<openni::VideoStream> getDepthVideoStream() = 0;

  std::string name() const {
    return name_;
  }


private:
  std::map<CloudDataSource::State, std::string> kStateStrings = {
    { STOPPED,    "STOPPED" },
    { STARTING,   "STARTING" },
    { RUNNING,    "RUNNING" },
    { RECORDING,  "RECORDING" },
    { STOPPING,   "STOPPING" }
  };

  const std::string name_;
  State state_;

  std::thread worker_;
  std::thread stop_worker_;

  openni::Recorder recorder_;

  FpsCounter fps_counter_;
  float fps_;

  void updateFps(const FpsCounter::Event& event);
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_CLOUDDATASOURCE_H
