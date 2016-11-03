//
// Created by Masayuki IZUMI on 10/30/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_CAPTUREDLOGLOADER_H
#define CIPOINTCLOUDVIEWERAPP_CAPTUREDLOGLOADER_H

#include <atomic>
#include <map>
#include <string>
#include <thread>

#include "CapturedLog.h"
#include "Skeleton.h"

namespace io {

class CapturedLogLoader {
public:
  struct CompleteLoadingAction {
    std::string serial;
  };

  CapturedLogLoader(
    const std::string &dir,
    const std::string &serial,
    const int user_count
  );
  ~CapturedLogLoader();

  size_t total_file_count() const {
    return total_file_count_;
  }

  size_t loaded_file_count() const {
    return loaded_file_count_;
  }

  int64_t started_at() const {
    return started_at_;
  }

  int64_t ended_at() const {
    return ended_at_;
  }

  bool hasLoaded() const {
    return loaded_;
  }


private:
  const std::string serial_;
  const int user_count_;
  size_t total_file_count_;
  size_t loaded_file_count_;
  int64_t started_at_;
  int64_t ended_at_;
  std::map<int, std::map<int64_t, std::string>> pcd_files_;
  std::map<int64_t, SkeletonsPtr> skeletons_;
  std::map<int, std::shared_ptr<CapturedLog>> logs_;

  std::atomic<bool> loaded_;
  std::thread loader_;


  void initialize(const std::string &dir);
  void initializePcdFileList(const std::string &dir);
  void loadAllSkeletons(const std::string &dir);
  void loadAllClouds();
  SkeletonsPtr loadSkeletons(const std::string &path);
  CloudPtr loadCloud(const std::string &path, int user_id, int64_t timestamp);
};

}

#endif //CIPOINTCLOUDVIEWERAPP_CAPTUREDLOGLOADER_H
