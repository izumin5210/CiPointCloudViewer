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

#include "PointCloudGrabber.hpp"

namespace grabber {

class PcdGrabber : public PointCloudGrabber {
public:
    using PointCloudGrabber::start;

    PcdGrabber(const bpath path)
        : PointCloudGrabber(path)
    {
        pcl::io::loadPCDFile(path_.string(), *cloud_);
    }

    inline void start(bpt::ptime started_at, std::function<void()> &callback) override {
        callback();
    }

    inline void stop() override {
        // do nothing
    }
};

}

#endif /* PcdGrabber_hpp */
