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

#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>

namespace bpt = boost::posix_time;

namespace grabber {

class PointCloudGrabber {
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef boost::filesystem::path bpath;

    PointCloudGrabber(const bpath path)
        : path_(path)
        , cloud_(new PointCloud)
    {
    }

    inline bpath path() {
        return path_;
    }

    inline PointCloudPtr cloud() {
        return cloud_;
    }

    virtual inline void start(std::function<void()> &callback) {
        start(current_time(), callback);
    }

    virtual void start(bpt::ptime started_at, std::function<void()> &callback) = 0;

    virtual void stop() = 0;

protected:
    bpath path_;
    PointCloudPtr cloud_;

    inline bpt::ptime current_time() {
        return bpt::microsec_clock::local_time();
    }

    inline long long current_time_in_nanos() {
        return time_in_nanos(current_time());
    }

    inline long long time_in_nanos(bpt::ptime time) {
        return (time - bpt::ptime(boost::gregorian::date(1970, 1, 1))).total_nanoseconds();
    }
};

}

#endif /* PointCloudGrabber_hpp */
