//
// Created by Masayuki IZUMI on 10/30/16.
//

#include <pcl/io/pcd_io.h>

#include <msgpack.hpp>

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
  , total_file_count_ (0)
  , loaded_file_count_(0)
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
    initializePcdFileList(path.string());
    loadAllSkeletons((path / "skeletons").string());
    loadAllClouds();
    loaded_ = true;
    Signal<CompleteLoadingAction>::emit({serial_});
  });
}

void CapturedLogLoader::initializePcdFileList(const std::string &dir) {
  for (int i = 0; i < user_count_ + 1; i++) {
    auto path = (boost::filesystem::path(dir) / std::to_string(i)).string();
    util::eachFiles(path, [&](boost::filesystem::directory_entry &file) {
      if (util::hasExt(file.path(), ".pcd")) {
        auto stamp = std::stoll(util::basename(file.path()));
        if (stamp < started_at_) {
          started_at_ = stamp;
        }
        if (stamp > ended_at_) {
          ended_at_ = stamp;
        }
        pcd_files_[i][stamp] = file.path().string();
        total_file_count_++;
      }
    });
  }
}

void CapturedLogLoader::loadAllSkeletons(const std::string &dir) {
  util::eachFiles(dir, [&](boost::filesystem::directory_entry &file) {
    if (util::hasExt(file.path(), ".mpac")) {
      auto stamp = std::stoll(util::basename(file.path()));
      skeletons_[stamp] = loadSkeletons(file.path().string());
    }
  });
}

void CapturedLogLoader::loadAllClouds() {
  for (auto p1 : pcd_files_) {
    std::map<int64_t, CloudPtr> clouds;
    for (auto p2 : p1.second) {
      clouds[p2.first] = loadCloud(p2.second, p1.first, p2.first);
      loaded_file_count_++;
    }
    logs_[p1.first] = std::make_shared<CapturedLog>(serial_, p1.first, clouds);
  }
}

SkeletonsPtr CapturedLogLoader::loadSkeletons(const std::string &path) {
  char *buf;
  size_t size;
  std::ifstream ifs(path, std::ios::in | std::ios::binary);
  auto begin = static_cast<size_t>(ifs.tellg());
  ifs.seekg(0, ifs.end);
  auto end = static_cast<size_t>(ifs.tellg());
  ifs.clear();
  ifs.seekg(0, ifs.beg);
  size = end - begin;
  buf = new char[size];
  ifs.read(buf, size);
  msgpack::object_handle oh;
  msgpack::unpack(oh, buf, size);
  SkeletonsPtr skeletons(new Skeletons);
  oh.get().convert(*skeletons);
  return skeletons;
}

CloudPtr CapturedLogLoader::loadCloud(const std::string &path, int user_id, int64_t timestamp) {
  Cloud::PointCloudPtr cloud(new Cloud::PointCloud);
  pcl::io::loadPCDFile(path, *cloud);
  return std::make_shared<Cloud>(serial_, cloud, (*skeletons_[timestamp])[user_id], true);
}

}
