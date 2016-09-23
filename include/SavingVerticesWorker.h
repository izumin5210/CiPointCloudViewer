//
// Created by izumin on 16/08/07.
//

#ifndef CIPOINTCLOUDVIEWERAPP_SAVINGVERTICESWORKER_H
#define CIPOINTCLOUDVIEWERAPP_SAVINGVERTICESWORKER_H

#include "nod/nod.hpp"

#include "Dispatcher.h"
#include "Clouds.h"

#include <atomic>
#include <queue>

#include <boost/filesystem.hpp>

class SavingVerticesWorker {
public:
  virtual void start(std::string dir) = 0;
  virtual void stopSafety() = 0;
  virtual void stop() = 0;

  virtual size_t total_size() const = 0;
  virtual bool has_stopped() const = 0;
  virtual size_t size() const = 0;
  virtual float fps() const = 0;
};

fruit::Component<fruit::Required<Dispatcher>, SavingVerticesWorker>
getSavingVerticesWorker();

#endif //CIPOINTCLOUDVIEWERAPP_SAVINGVERTICESWORKER_H
