//
// Created by Masayuki IZUMI on 10/29/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_EXPORTER_H
#define CIPOINTCLOUDVIEWERAPP_EXPORTER_H

namespace io {
namespace exporter {

class Exporter {
public:
  virtual void start(const std::string &dir) = 0;
  virtual void stopSafety() = 0;
  virtual void stop() = 0;
  virtual bool hasStopped() const = 0;
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_EXPORTER_H
