//
//  PcdGrabber.hpp
//  CiPcdViewer
//
//  Created by Masayuki IZUMI on 7/1/16.
//
//

#ifndef PcdGrabber_hpp
#define PcdGrabber_hpp

#include <pcl/io/pcd_io.h>

#include "Clouds.h"
#include "Signal.h"
#include "model/CloudsManager.h"
#include "PointCloudGrabber.hpp"

namespace grabber {

class PcdGrabber : public PointCloudGrabber {
public:
  using PointCloudGrabber::start;

  PcdGrabber(const bpath path)
    : PointCloudGrabber(path)
  {
  }

  inline void start(bpt::ptime started_at) override {
    (void) started_at;
    pcl::io::loadPCDFile(path_.string(), *cloud_);
    Signal<Clouds::UpdateCloudAction>::emit({path_.string(), cloud_});
  }

  inline void stop() override {
    // do nothing
  }

  inline bool isPlaying() override {
    return true;
  }
};

}

#endif /* PcdGrabber_hpp */
