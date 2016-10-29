//
// Created by Masayuki IZUMI on 10/27/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_EXPORTERBASE_H
#define CIPOINTCLOUDVIEWERAPP_EXPORTERBASE_H

#include <atomic>
#include <chrono>
#include <queue>
#include <thread>

#include <boost/filesystem.hpp>

#include "Signal.h"
#include "FpsCounter.h"
#include "io/exporter/Exporter.h"

namespace io {
namespace exporter {

template <typename T>
class ExporterBase : public Exporter {
public:
  ExporterBase(const std::string key);
  ~ExporterBase();

  void start(const std::string &dir) override;
  void stopSafety() override;
  void stop() override;

  inline bool hasStopped() const override {
    return !acceptable_;
  }

  inline size_t total_size() const {
    return total_size_;
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

#endif //CIPOINTCLOUDVIEWERAPP_EXPORTERBASE_H
