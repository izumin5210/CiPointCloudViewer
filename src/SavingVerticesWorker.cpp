//
// Created by izumin on 16/08/07.
//

#include "SavingVerticesWorker.h"

#include "Signal.h"

#include <pcl/io/pcd_io.h>

SavingVerticesWorker::SavingVerticesWorker()
  : total_size_         (0)
  , worker_stopped_     (true)
  , vertices_acceptable_(false)
  , fps_                (0.0f)
{
  auto callback = std::bind(&SavingVerticesWorker::onVerticesUpdate, this, std::placeholders::_1);
  Signal<Clouds::UpdateVerticesAction>::connect(callback);
  Signal<FpsCounter::Event>::connect(std::bind(&SavingVerticesWorker::onFpsUpdate, this, std::placeholders::_1));
}

SavingVerticesWorker::~SavingVerticesWorker() {
  stop();
}

void SavingVerticesWorker::start(std::string dir) {
  auto started_at = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  std::stringstream ss_dir;
  ss_dir << dir << "/" << started_at;
  dir_ = boost::filesystem::path(ss_dir.str());
  boost::filesystem::create_directory(dir_);
  if (worker_stopped_) {
    worker_ = std::thread([&]{
      fps_counter_.start(kFpsCounterKey);
      worker_stopped_ = false;
      vertices_acceptable_ = true;
      while (!worker_stopped_ || !queue_.empty()) {
        if (!queue_.empty()) {
          auto item = queue_.front();
          auto stamp = std::chrono::duration_cast<std::chrono::milliseconds>(item.timestamp.time_since_epoch()).count();
          std::stringstream ss;
          ss << item.key << "/" << stamp << ".pcd";
          auto path = dir_ / boost::filesystem::path(ss.str());
          if (!boost::filesystem::exists(path.parent_path())) {
            boost::filesystem::create_directory(path.parent_path());
          }
          pcl::PointCloud<pcl::PointXYZRGBA> cloud;
          for (auto v : *item.vertices) {
            pcl::PointXYZRGBA p;
            p.x = v.xyz[0];
            p.y = v.xyz[1];
            p.z = v.xyz[2];
            p.r = v.rgb[0];
            p.g = v.rgb[1];
            p.b = v.rgb[2];
            cloud.push_back(p);
          }
          pcl::io::savePCDFile(path.string(), cloud);
          queue_.pop();
        }
        fps_counter_.passFrame();
      }
      stop();
    });
  } else {
    vertices_acceptable_ = true;
  }
}

void SavingVerticesWorker::stop() {
  stopSafety();
  worker_stopped_ = true;
  if (worker_.joinable()) {
    worker_.join();
  }
  fps_counter_.stop();
}

void SavingVerticesWorker::stopSafety() {
  vertices_acceptable_ = false;
}

void SavingVerticesWorker::onVerticesUpdate(const Clouds::UpdateVerticesAction &action) {
  if (vertices_acceptable_) {
    queue_.push({action.key, action.timestamp, action.vertices});
    total_size_++;
  }
}

void SavingVerticesWorker::onFpsUpdate(const FpsCounter::Event &event) {
  if (event.key == kFpsCounterKey) {
    fps_ = event.fps;
  }
}
