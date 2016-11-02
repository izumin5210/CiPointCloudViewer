//
// Created by Masayuki IZUMI on 10/30/16.
//

#include "yaml-cpp/yaml.h"

#include "Signal.h"
#include "io/CapturedLogManager.h"
#include "util/util.h"

namespace io {

void CapturedLogManager::load(const std::string &path) {
  Signal<OpenLogAction>::emit({ path });
}

CapturedLogManager::CapturedLogManager()
  : state_                  (State::NO_LOGS)
  , started_at_             (INT64_MAX)
  , ended_at_               (INT64_MIN)
  , started_at_in_real_     (0)
  , stopped_at_in_real_     (0)
  , player_worker_canceled_ (true)
{
  Signal<OpenLogAction>::connect(
    std::bind(&CapturedLogManager::onLogOpen, this, std::placeholders::_1)
  );
  Signal<CapturedLogLoader::CompleteLoadingAction>::connect(
    std::bind(&CapturedLogManager::onLoadingComplete, this, std::placeholders::_1)
  );
}

CapturedLogManager::~CapturedLogManager() {
  stop();
}

void CapturedLogManager::start() {
  if (state_ != State::LOADED) { return; }
  if (player_worker_.joinable()) {
    player_worker_.join();
  }
  player_worker_ = std::thread([&] {
    if (stopped_at_in_real_ == 0) {
      started_at_in_real_ = 0;
      Signal<CapturedLog::ResetTimestampAction>::emit({});
    }
    for (auto pair : loaders_) {
      if (started_at_ > pair.second->started_at()) {
        started_at_ = pair.second->started_at();
      }
      if (ended_at_ < pair.second->ended_at()) {
        ended_at_ = pair.second->ended_at();
      }
    }
    player_worker_canceled_ = false;
    started_at_in_real_ += util::to_us(util::now()) - stopped_at_in_real_;
    state_ = State::PLAYING;
    while (!player_worker_canceled_) {
      auto now = util::to_us(util::now());
      auto duration = now - started_at_in_real_;
      if ((ended_at_ - started_at_) < duration) {
        player_worker_canceled_ = true;
        stopped_at_in_real_ = 0;
      } else {
        util::sleep(4);
        Signal<CapturedLog::UpdateTimestampAction>::emit({ started_at_ + duration });
      }
    }
    state_ = State::LOADED;
  });
}

void CapturedLogManager::stop() {
  player_worker_canceled_ = true;
  if (player_worker_.joinable()) {
    player_worker_.join();
  }
  stopped_at_in_real_ = util::to_us(util::now());
  state_ = State::LOADED;
}

void CapturedLogManager::onLogOpen(const OpenLogAction &action) {
  if (util::exists(action.path)) {
    // TODO: should handle parse errors.
    auto dir = boost::filesystem::path(action.path).parent_path().string();
    auto yaml = YAML::LoadFile(action.path);
    auto cameras = yaml["cameras"];
    state_ = State::LOADING;
    for (size_t i = 0; i < cameras.size(); i++) {
      auto camera = cameras[i];
      auto serial = camera["serial"].as<std::string>();
      auto user_count = camera["user_count"].as<int>();
      loaders_[serial] = std::make_unique<CapturedLogLoader>(dir, serial, user_count);
    }
  }
}

void CapturedLogManager::onLoadingComplete(const CapturedLogLoader::CompleteLoadingAction &action) {
  loaded_log_serials_.emplace(action.serial);
  if (loaders_.size() == loaded_log_serials_.size() && state_ == CapturedLogManager::State::LOADING) {
    state_ = State::LOADED;
  }
}

}
