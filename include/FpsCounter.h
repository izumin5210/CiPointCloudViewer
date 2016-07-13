//
// Created by Masayuki IZUMI on 7/13/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_FPSCOUNTER_H
#define CIPOINTCLOUDVIEWERAPP_FPSCOUNTER_H

#include <iostream>
#include <chrono>

class FpsCounter {
public:
  using time_point = std::chrono::system_clock::time_point;

  struct Event {
    std::string key;
    float fps;
  };

  FpsCounter(int interval = 10);

  void start(std::string key);
  void passFrame();


private:
  const int interval_;

  std::string key_;
  time_point prev_;
  time_point now_;
  float fps_;
  int count_;

  time_point now();
  void emitSignal();
};

#endif //CIPOINTCLOUDVIEWERAPP_FPSCOUNTER_H
