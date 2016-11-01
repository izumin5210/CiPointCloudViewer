//
// Created by Masayuki IZUMI on 10/29/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_CAPTUREDLOGMANAGER_H
#define CIPOINTCLOUDVIEWERAPP_CAPTUREDLOGMANAGER_H

#include <set>

#include "CapturedLogLoader.h"

namespace io {

class CapturedLogManager {
public:
  struct OpenLogAction {
    std::string path;
  };

  enum struct State {
    NO_LOGS,
    LOADING,
    LOADED,
    PLAYING
  };

  static void load(const std::string &path);

  CapturedLogManager();
  ~CapturedLogManager();

  void start();
  void stop();

  inline State state() const {
    return state_;
  }

  std::map<std::string, std::shared_ptr<CapturedLogLoader>> loaders() const {
    return loaders_;
  }


private:
  std::map<std::string, std::shared_ptr<CapturedLogLoader>> loaders_;
  std::set<std::string> loaded_log_serials_;
  State state_;
  int64_t started_at_;
  int64_t ended_at_;
  int64_t started_at_in_real_;
  std::thread player_worker_;
  std::atomic<bool> player_worker_canceled_;
  std::atomic<bool> player_waited_;

  void onLogOpen(const OpenLogAction &action);
  void onLoadingComplete(const CapturedLogLoader::CompleteLoadingAction &action);
};

}

#endif //CIPOINTCLOUDVIEWERAPP_CAPTUREDLOGMANAGER_H
