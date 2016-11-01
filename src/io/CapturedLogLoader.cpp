//
// Created by Masayuki IZUMI on 10/30/16.
//

#include <pcl/io/pcd_io.h>

#include "Cloud.h"
#include "Signal.h"
#include "io/CapturedLogLoader.h"
#include "util/util.h"

namespace io {

CapturedLogLoader::CapturedLogLoader(
  const std::string &dir,
  const std::string &serial,
  const int user_count
)
  : serial_     (serial)
  , user_count_ (user_count)
  , started_at_ (INT64_MAX)
  , ended_at_   (INT64_MIN)
  , loaded_     (false)
{
  initialize(dir);
}

CapturedLogLoader::~CapturedLogLoader() {
  if (loader_.joinable()) {
    loader_.join();
  }
}

void CapturedLogLoader::initialize(const std::string &dir) {
  loader_ = std::thread([&] {
    auto path = boost::filesystem::path(dir) / serial_;
    // FIXME: Constantize directory names and extensions
    for (int i = 0; i < user_count_ + 1; i++) {
      util::eachFiles((path / std::to_string(i)).string(), [&](boost::filesystem::directory_entry &file) {
        if (util::hasExt(file.path(), ".pcd")) {
          auto stamp = std::stoll(util::basename(file.path()));
          if (stamp < started_at_) {
            started_at_ = stamp;
          }
          if (stamp > ended_at_) {
            ended_at_ = stamp;
          }
          pcd_files_[i][stamp] = file.path().string();
        }
      });
    }
    util::eachFiles((path / "skeletons").string(), [&](boost::filesystem::directory_entry &file) {
      if (util::hasExt(file.path(), ".mpac")) {
        auto stamp = std::stoll(util::basename(file.path()));
        skeleton_files_[stamp] = file.path().string();
      }
    });
    for (auto p1 : pcd_files_) {
      std::map<int64_t, CloudPtr> clouds;
      for (auto p2 : p1.second) {
        // TODO: Update loading progress status
        Cloud::PointCloudPtr cloud(new Cloud::PointCloud);
        pcl::io::loadPCDFile(p2.second, *cloud);
        clouds[p2.first] = std::make_shared<Cloud>(serial_, cloud, true);
      }
      logs_[p1.first] = std::make_shared<CapturedLog>(serial_, p1.first, clouds);
    }
    loaded_ = true;
    Signal<CompleteLoadingAction>::emit({serial_});
  });
}

}
