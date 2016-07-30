//
// Created by Masayuki IZUMI on 7/20/16.
//

#include "Signal.h"
#include "io/SequentialPcdPlayer.h"

namespace io {

SequentialPcdPlayer::SequentialPcdPlayer(const std::string path)
  : path_(bpath(path))
  , cloud_(new PointCloud)
  , loading_progress_(0, 0)
  , loader_worker_canceled_(true)
  , player_worker_canceled_(true)
{
  initialize();
}

SequentialPcdPlayer::~SequentialPcdPlayer() {
  stop();
  loader_worker_canceled_ = true;
  if (loader_worker_.joinable()) {
    loader_worker_.join();
  }
}

void SequentialPcdPlayer::start() {
  start(files_.begin()->first);
}

void SequentialPcdPlayer::start(bptime started_at) {
  start(time_in_nanos(started_at));
}

void SequentialPcdPlayer::stop() {
  player_worker_canceled_ = true;
  if (player_worker_.joinable()) {
    player_worker_.join();
  }
}

void SequentialPcdPlayer::initialize() {
  loader_worker_canceled_ = false;
  loader_worker_ = std::thread([&]{
    for (auto file : boost::make_iterator_range(ditr(path_), ditr())) {
      if (file.path().extension().string() == ".pcd") {
        const auto stamp = boost::posix_time::from_iso_string(file.path().stem().string());
        files_[time_in_nanos(stamp)] = file.path();
      }
    }
    unsigned long i = 0;
    loading_progress_ = glm::vec2(0, files_.size());
    for (auto pair : files_) {
      if (loader_worker_canceled_) { break; }
      const auto cloud = PointCloudPtr(new PointCloud);
      pcl::io::loadPCDFile(pair.second.string(), *cloud);
      clouds_[pair.first] = cloud;
      i++;
      loading_progress_ = glm::vec2(i, files_.size());
    }
  });
}

void SequentialPcdPlayer::start(const uint64_t started_at) {
  player_worker_canceled_ = false;
  player_worker_ = std::thread([this, started_at] {
    auto itr = files_.begin();
    started_at_in_real_ = time_in_nanos(current_time());
    waiting_ = false;
    while (!player_worker_canceled_) {
      if (!isLoaded(itr->first)) {
        if (waiting_) {
          waited_since_ = current_time_in_nanos();
        } else {
          waiting_ = true;
        }
        continue;
      } else if (waiting_) {
        started_at_in_real_ += current_time_in_nanos() - waited_since_;
        waiting_ = false;
      }
      if ((itr->first - started_at) <= elapsedTime()) {
        cloud_ = clouds_[itr->first];
        // TODO
//        Signal<Clouds::UpdateCloudAction>::emit({path_.string(), cloud_});
        itr++;
      }
      if (itr == files_.end()) {
        itr = files_.begin();
        started_at_in_real_ = time_in_nanos(current_time());
      }
    }
  });
}

}
