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

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "cinder/gl/gl.h"

#endif /* Cloud_hpp */

class CloudGl {
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    
    CloudGl(const boost::filesystem::path path)
        : _path(path)
        , _cloud_input(new PointCloud)
        , _cloud_filtered(new PointCloud)
        , _batch(ci::gl::VertBatch::create(GL_POINTS))
    {
        pcl::io::loadPCDFile(path.string(), *_cloud_input);
    }
    
    void filter(std::function<void (PointCloudPtr&)> fun) {
        pcl::copyPointCloud(*_cloud_input, *_cloud_filtered);
        fun(_cloud_filtered);
    }
    
    void update() {
        _batch->clear();
        for (auto point : _cloud_filtered->points) {
            _batch->color(ci::ColorA8u(point.r, point.g, point.b, point.a));
            _batch->vertex(ci::vec3(point.x, point.y, point.z));
        }
    }
    
    void draw() {
        _batch->draw();
    }
    
private:
    const boost::filesystem::path _path;
    PointCloudPtr _cloud_input;
    PointCloudPtr _cloud_filtered;
    
    ci::gl::VertBatchRef _batch;
};  