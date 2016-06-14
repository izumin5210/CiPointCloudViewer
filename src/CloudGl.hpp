//
//  Cloud.hpp
//  CiPcdViewer
//
//  Created by Masayuki IZUMI on 6/12/16.
//
//

#ifndef Cloud_hpp
#define Cloud_hpp

#include <stdio.h>
#include <sstream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "cinder/params/Params.h"
#include "cinder/gl/gl.h"

#endif /* Cloud_hpp */

class CloudGl {
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;

    CloudGl(const boost::filesystem::path path)
        : path_(path)
        , cloud_(new PointCloud)
    {
        pcl::io::loadPCDFile(path_.string(), *cloud_);
    }

    PointCloudPtr cloud() {
        return cloud_;
    }

private:
    const boost::filesystem::path path_;
    PointCloudPtr cloud_;
};