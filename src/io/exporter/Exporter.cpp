//
// Created by Masayuki IZUMI on 10/27/16.
//

#include "io/exporter/Exporter.h"
#include "io/exporter/SkeletonsExporter.h"

namespace io {
namespace exporter {

template <typename T>
Exporter<T>::Exporter(const std::string key)
  : key_            (key)
  , total_size_     (0)
  , worker_stopped_ (true)
  , acceptable_     (false)
  , fps_            (0)
{
  Signal<T>::connect(std::bind(&Exporter<T>::onItemUpdate, this, std::placeholders::_1));
  Signal<FpsCounter::Event>::connect(std::bind(&Exporter<T>::onFpsUpdate, this, std::placeholders::_1));
}

template <typename T>
Exporter<T>::~Exporter() {
  stop();
}

template <typename T>
void Exporter<T>::start(std::string dir) {
  dir_  = boost::filesystem::path(dir);
  if (worker_stopped_) {
    worker_ = std::thread([&] {
      fps_counter_.start(key_);
      worker_stopped_ = false;
      acceptable_ = true;
      while (!worker_stopped_ || !queue_.empty()) {
        if (!queue_.empty()) {
          save(queue_.front());
          queue_.pop();
        }
        fps_counter_.passFrame();
      }
      stop();
    });
  } else {
    acceptable_ = true;
  }
}

template <typename T>
void Exporter<T>::stopSafety() {
  acceptable_ = false;
}

template <typename T>
void Exporter<T>::stop() {
  stopSafety();
  worker_stopped_ = true;
  if (worker_.joinable()) {
    worker_.join();
  }
  fps_counter_.stop();
}

template <typename T>
void Exporter<T>::onItemUpdate(const T &item) {
  add(item);
}

template <typename T>
void Exporter<T>::onFpsUpdate(const FpsCounter::Event &event) {
  if (event.key == key_) {
    fps_ = event.fps;
  }
}

template class Exporter<Clouds::UpdateSkeletonsAction>;

}
}
