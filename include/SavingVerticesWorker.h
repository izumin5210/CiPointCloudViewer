//
// Created by izumin on 16/08/07.
//

#ifndef CIPOINTCLOUDVIEWERAPP_SAVINGVERTICESWORKER_H
#define CIPOINTCLOUDVIEWERAPP_SAVINGVERTICESWORKER_H

#include "nod/nod.hpp"

#include "Clouds.h"
#include "FpsCounter.h"

#include <atomic>
#include <queue>

#include <boost/filesystem.hpp>

class SavingVerticesWorker {
public:
  SavingVerticesWorker(const std::shared_ptr<Clouds> &clouds);
  ~SavingVerticesWorker();

  void start(std::string dir);
  void stopSafety();
  void stop();

  inline size_t total_size() const {
    return total_size_;
  }

  inline bool has_stopped() const {
    return !vertices_acceptable_;
  }

  inline size_t size() const {
    return queue_.size();
  }

  inline float fps() const {
    return fps_;
  }

private:
  struct QueueItem {
    std::string key;
    std::chrono::system_clock::time_point timestamp;
    VerticesPtr vertices;
    int user_count;
  };

  const std::string kFpsCounterKey = "SavingVerticesWorker";

  const std::shared_ptr<Clouds> clouds_;

  std::queue<QueueItem> queue_;
  std::atomic<size_t> total_size_;

  boost::filesystem::path dir_;

  std::thread worker_;
  std::atomic<bool> worker_stopped_;
  std::atomic<bool> vertices_acceptable_;

  FpsCounter fps_counter_;
  float fps_;

  void onVerticesUpdate(const Clouds::UpdateVerticesAction &action);
  void onFpsUpdate(const FpsCounter::Event &event);
};

#endif //CIPOINTCLOUDVIEWERAPP_SAVINGVERTICESWORKER_H
