//
// Created by Masayuki IZUMI on 10/27/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_EXPORTER_H
#define CIPOINTCLOUDVIEWERAPP_EXPORTER_H

#include <atomic>
#include <chrono>
#include <queue>
#include <thread>

#include <boost/filesystem.hpp>

#include "Signal.h"
#include "FpsCounter.h"

namespace io {
namespace exporter {

template <typename T>
class Exporter {
public:
  Exporter(const std::string key);
  ~Exporter();

  void start(std::string dir);
  void stopSafety();
  void stop();

  inline size_t total_size() const {
    return total_size_;
  }

  inline bool has_stopped() const {
    return !acceptable_;
  }

  inline size_t size() const {
    return queue_.size();
  }

  inline float fps() const {
    return fps_;
  }


protected:
  virtual void save(const T &item) = 0;

  inline boost::filesystem::path dir() const {
    return dir_;
  }

  inline void add(const T &item) {
    if (acceptable_) {
      queue_.push(item);
      total_size_++;
    }
  }


private:
  const std::string key_;
  std::queue<T> queue_;
  std::atomic<size_t> total_size_;

  boost::filesystem::path dir_;

  std::thread worker_;
  std::atomic<bool> worker_stopped_;
  std::atomic<bool> acceptable_;

  FpsCounter fps_counter_;
  float fps_;

  void onItemUpdate(const T &item);
  void onFpsUpdate(const FpsCounter::Event &event);
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_EXPORTER_H
