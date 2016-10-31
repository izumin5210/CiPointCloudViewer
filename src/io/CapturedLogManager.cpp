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
{
  auto cb =std::bind(&CapturedLogManager::onLogOpen, this, std::placeholders::_1);
  Signal<OpenLogAction>::connect(cb);
}

void CapturedLogManager::onLogOpen(const OpenLogAction &action) {
  if (util::exists(action.path)) {
    // TODO: should handle parse errors.
    auto dir = boost::filesystem::path(action.path).parent_path().string();
    auto yaml = YAML::LoadFile(action.path);
    auto cameras = yaml["cameras"];
    for (size_t i = 0; i < cameras.size(); i++) {
      auto camera = cameras[i];
      auto serial = camera["serial"].as<std::string>();
      auto user_count = camera["user_count"].as<int>();
      // TODO: Load logs
    }
  }
}

}
