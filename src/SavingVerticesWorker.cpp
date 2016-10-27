//
// Created by izumin on 16/08/07.
//

#include "SavingVerticesWorker.h"

#include "Signal.h"
#include "util/util.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

SavingVerticesWorker::SavingVerticesWorker(const std::shared_ptr<Clouds> &clouds)
  : clouds_(clouds)
  , total_size_         (0)
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
  dir_  = boost::filesystem::path(dir);
  if (worker_stopped_) {
    worker_ = std::thread([&]{
      fps_counter_.start(kFpsCounterKey);
      worker_stopped_ = false;
      vertices_acceptable_ = true;
      while (!worker_stopped_ || !queue_.empty()) {
        if (!queue_.empty()) {
          const auto item = queue_.front();
          const size_t n = static_cast<size_t>(item.user_count + 1);
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr *clouds = new pcl::PointCloud<pcl::PointXYZRGBA>::Ptr[n];

          auto stamp = std::chrono::duration_cast<std::chrono::microseconds>(item.timestamp.time_since_epoch()).count();
          boost::system::error_code error;

          for (size_t i = 0; i < n; i++) {
            auto d = dir_ / item.key / std::to_string(i);
            if (!boost::filesystem::exists(d)) {
              util::checkStatus(boost::filesystem::create_directories(d, error), "Failed to create directory.");
            }
            clouds[i] = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
          }

          auto calib_params = clouds_->calib_params_map()[item.key];

          for (auto v : *item.vertices) {
            pcl::PointXYZRGBA p;
            p.z = v.xyz[2] / 1000;
            p.x = (v.xyz[0] - calib_params.cx) * p.z / calib_params.fx;
            p.y = (v.xyz[1] - calib_params.cy) * p.z / calib_params.fy;
            p.r = v.rgb[0];
            p.g = v.rgb[1];
            p.b = v.rgb[2];
            clouds[v.user_id]->push_back(p);
          }

          for (size_t i = 0; i < n; i++) {
            if (clouds[i]->empty()) {
              pcl::PointXYZRGBA p;
              p.x = p.y = p.z = p.r = p.g = p.b = p.a = 0;
              clouds[i]->push_back(p);
            }
            auto d = dir_ / item.key / std::to_string(i);
            pcl::PointCloud<pcl::PointXYZRGBA> cloud;
            pcl::transformPointCloud(*clouds[i], cloud, calib_params.calib_matrix);
            pcl::io::savePCDFileBinary((d / (std::to_string(stamp) + ".pcd")).string(), cloud);
          }

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
    queue_.push({action.key, action.timestamp, action.vertices, action.user_count});
    total_size_++;
  }
}

void SavingVerticesWorker::onFpsUpdate(const FpsCounter::Event &event) {
  if (event.key == kFpsCounterKey) {
    fps_ = event.fps;
  }
}
