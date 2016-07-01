//
//  PcdGrabber.hpp
//  CiPcdViewer
//
//  Created by Masayuki IZUMI on 7/1/16.
//
//

#ifndef PcdGrabber_hpp
#define PcdGrabber_hpp

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "PointCloudGrabber.hpp"

namespace grabber {

class PcdGrabber : public PointCloudGrabber {
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
            
    PcdGrabber(const boost::filesystem::path path)
        : PointCloudGrabber(path)
    {
        pcl::io::loadPCDFile(path_.string(), *cloud_);
    }

    inline void start(std::function<void()> callback) {
        callback();
    }
};

}

#endif /* PcdGrabber_hpp */
