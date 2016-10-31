//
// Created by Masayuki IZUMI on 10/30/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_CAPTUREDLOGLOADER_H
#define CIPOINTCLOUDVIEWERAPP_CAPTUREDLOGLOADER_H

#include <atomic>
#include <map>
#include <string>
#include <thread>

namespace io {

class CapturedLogLoader {
public:
  CapturedLogLoader(
    const std::string &dir,
    const std::string &serial,
    const int user_count
  );
  ~CapturedLogLoader();

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
  int64_t started_at_;
  int64_t ended_at_;
  std::map<int, std::map<int64_t, std::string>> pcd_files_;
  std::map<int64_t, std::string> skeleton_files_;

  std::atomic<bool> loaded_;
  std::thread loader_;


  void initialize(const std::string &dir);
};

}

#endif //CIPOINTCLOUDVIEWERAPP_CAPTUREDLOGLOADER_H
