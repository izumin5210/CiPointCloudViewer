//
// Created by Masayuki IZUMI on 7/13/16.
//

#include "FpsCounter.h"
#include "Signal.h"

FpsCounter::FpsCounter(int interval)
  : interval_ (interval)
  , prev_     (now())
  , now_      (now())
  , fps_      (0.0f)
  , count_    (0)
{
}

void FpsCounter::start(std::string key) {
  key_  = key;
  prev_ = now();
  now_  = prev_;
}

void FpsCounter::stop() {
  fps_ = 0;
  emitSignal();
}

void FpsCounter::passFrame() {
  now_ = now();
  if (++count_ == interval_) {
    count_ = 0;
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now_ - prev_).count();
    fps_ = ((float) interval_) / elapsed * 1000;
    prev_ = now_;
    emitSignal();
  }
}

std::chrono::system_clock::time_point FpsCounter::now() {
  return std::chrono::system_clock::now();
}

void FpsCounter::emitSignal() {
  Signal<Event>::emit({key_, fps_});
}
