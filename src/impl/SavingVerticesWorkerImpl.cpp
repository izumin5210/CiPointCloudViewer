//
// Created by izumin on 16/08/07.
//

#include <pcl/io/pcd_io.h>

#include "impl/SavingVerticesWorkerImpl.h"
#include "action/CloudsAction.h"
#include "FpsCounter.h"

class SavingVerticesWorkerImpl : public SavingVerticesWorker {
public:
  INJECT(SavingVerticesWorkerImpl(
      std::shared_ptr<Dispatcher> dispatcher
  ))
    : dispatcher_(dispatcher)
    , total_size_         (0)
    , worker_stopped_     (true)
    , vertices_acceptable_(false)
    , fps_                (0.0f)
  {
    initialize();
  }

  ~SavingVerticesWorkerImpl() {
    stop();
  }

  void start(std::string dir) override {
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
            for (auto v : item.vertices) {
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

  void stop() override {
    stopSafety();
    worker_stopped_ = true;
    if (worker_.joinable()) {
      worker_.join();
    }
    fps_counter_.stop();
  }

  void stopSafety() override {
    vertices_acceptable_ = false;
  }

  size_t total_size() const override {
    return total_size_;
  }

  bool has_stopped() const override {
    return !vertices_acceptable_;
  }

  size_t size() const override {
    return queue_.size();
  }

  float fps() const override {
    return fps_;
  }


private:
  struct QueueItem {
    std::string key;
    std::chrono::system_clock::time_point timestamp;
    Vertices vertices;
  };

  const std::string kFpsCounterKey = "SavingVerticesWorker";

  std::shared_ptr<Dispatcher> dispatcher_;
  std::queue<QueueItem> queue_;
  std::atomic<size_t> total_size_;

  boost::filesystem::path dir_;

  std::thread worker_;
  std::atomic<bool> worker_stopped_;
  std::atomic<bool> vertices_acceptable_;

  FpsCounter fps_counter_;
  float fps_;

  void initialize() {
    auto callback = std::bind(&SavingVerticesWorkerImpl::onVerticesUpdate, this, std::placeholders::_1);
    dispatcher_->connect<UpdateVerticesAction>(callback);
    dispatcher_->connect<FpsCounter::Event>(std::bind(&SavingVerticesWorkerImpl::onFpsUpdate, this, std::placeholders::_1));
  }

  void onVerticesUpdate(const UpdateVerticesAction &action) {
    if (vertices_acceptable_) {
      queue_.push({action.key, action.timestamp, action.vertices});
      total_size_++;
    }
  }

  void onFpsUpdate(const FpsCounter::Event &event) {
    if (event.key == kFpsCounterKey) {
      fps_ = event.fps;
    }
  }
};

fruit::Component<fruit::Required<Dispatcher>, SavingVerticesWorker>
getSavingVerticesWorkerImplComponent() {
  return fruit::createComponent()
      .bind<SavingVerticesWorker, SavingVerticesWorkerImpl>();
}
