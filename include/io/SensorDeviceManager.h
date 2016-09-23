//
//  SensorDeviceManager.hpp
//  CiPointCloudViewer
//
//  Created by Masayuki IZUMI on 7/8/16.
//
//

#ifndef SensorDeviceManager_hpp
#define SensorDeviceManager_hpp

#include "SensorDevice.hpp"

namespace io {

class SensorDeviceManager {
public:
  virtual std::map<std::string, std::shared_ptr<SensorDevice>> devices() = 0;
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual void refresh() = 0;
  virtual void open(const std::string uri) = 0;
};

}

#endif /* SensorDeviceManager_hpp */
