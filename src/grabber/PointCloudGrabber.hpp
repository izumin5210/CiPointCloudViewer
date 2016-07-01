//
//  PointCloudGrabber.hpp
//  CiPcdViewer
//
//  Created by Masayuki IZUMI on 7/1/16.
//
//

#ifndef PointCloudGrabber_hpp
#define PointCloudGrabber_hpp

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace grabber {

class PointCloudGrabber {
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;

    PointCloudGrabber(const boost::filesystem::path path)
        : path_(path)
        , cloud_(new PointCloud)
    {
    }

    inline boost::filesystem::path path() {
        return path_;
    }

    inline PointCloudPtr cloud() {
        return cloud_;
    }

    virtual void start(std::function<void()> &callback) {
    }

protected:
    boost::filesystem::path path_;
    PointCloudPtr cloud_;
};

}

#endif /* PointCloudGrabber_hpp */
