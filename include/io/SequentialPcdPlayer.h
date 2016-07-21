//
// Created by Masayuki IZUMI on 7/20/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_SEQUENTIALPCDPLAYER_H
#define CIPOINTCLOUDVIEWERAPP_SEQUENTIALPCDPLAYER_H

#include <pcl/io/pcd_io.h>

#include <thread>
#include <map>

#include "glm/glm.hpp"
#include "cinder/Signals.h"

#include "Clouds.h"

namespace io {

class SequentialPcdPlayer {
public:
  using PointT        = pcl::PointXYZRGBA;
  using PointCloud    = pcl::PointCloud<PointT>;
  using PointCloudPtr = PointCloud::Ptr;

  using ditr    = boost::filesystem::directory_iterator;
  using bpath   = boost::filesystem::path;
  using bptime  = boost::posix_time::ptime;

  SequentialPcdPlayer(const std::string path);
  ~SequentialPcdPlayer();

  void start();
  void start(bptime started_at);
  void stop();

  inline bool isPlaying() {
    return !player_worker_canceled_;
  }

  inline bool isLoaded(const uint64_t started_at) {
    return clouds_.find(started_at) != clouds_.end();
  }

  inline uint64_t elapsedTime() {
    return current_time_in_nanos() - started_at_in_real_;
  }

  inline glm::vec2 loading_progress() {
    return loading_progress_;
  }


private:
  bpath path_;
  PointCloudPtr cloud_;

  std::map<uint64_t, PointCloudPtr> clouds_;
  std::map<uint64_t, bpath> files_;

  glm::vec2 loading_progress_;

  std::atomic<bool> waiting_;

  std::thread loader_worker_;
  std::atomic<bool> loader_worker_canceled_;
  std::thread player_worker_;
  std::atomic<bool> player_worker_canceled_;

  std::atomic<uint64_t> started_at_in_real_;
  std::atomic<uint64_t> waited_since_;

  void initialize();
  void start(const uint64_t started_at);

  inline bptime current_time() {
    return boost::posix_time::microsec_clock::local_time();
  }

  inline long long current_time_in_nanos() {
    return time_in_nanos(current_time());
  }

  inline long long time_in_nanos(bptime time) {
    return (time - boost::posix_time::ptime(boost::gregorian::date(1970, 1, 1))).total_nanoseconds();
  }
};

}

#endif //CIPOINTCLOUDVIEWERAPP_SEQUENTIALPCDPLAYER_H
