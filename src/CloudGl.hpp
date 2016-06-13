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
        : _path(path)
        , _cloud_input(new PointCloud)
        , _cloud_filtered(new PointCloud)
        , _batch(ci::gl::VertBatch::create(GL_POINTS))
        , _params(ci::params::InterfaceGl::create(path.filename().string(), ci::ivec2(200, 100)))
    {
        pcl::io::loadPCDFile(path.string(), *_cloud_input);
        _params->addText("cloud_size", "label=`Cloud Size: 0`");
        _params->addText("filtered_cloud_size", "label=`Filtered: 0`");
        _params->addParam("Visible", &_visible);
        _params->minimize();
    }

    void filter(std::function<void (PointCloudPtr&)> fun) {
        pcl::copyPointCloud(*_cloud_input, *_cloud_filtered);
        fun(_cloud_filtered);

        std::stringstream ss_input;
        ss_input << "label=`Cloud Size: " << _cloud_input->size() << "`";
        _params->setOptions("cloud_size", ss_input.str());

        std::stringstream ss_filtered;
        ss_filtered << "label=`Filtered: " << _cloud_filtered->size() << "`";
        _params->setOptions("filtered_cloud_size", ss_filtered.str());
    }

    void update() {
        _batch->clear();
        for (auto point : _cloud_filtered->points) {
            _batch->color(ci::ColorA8u(point.r, point.g, point.b, point.a));
            _batch->vertex(ci::vec3(point.x, point.y, point.z));
        }
    }

    void draw() {
        if (_visible) {
            _batch->draw();
        }
        _params->draw();
    }

private:
    const boost::filesystem::path _path;
    PointCloudPtr _cloud_input;
    PointCloudPtr _cloud_filtered;

    ci::gl::VertBatchRef _batch;
    ci::params::InterfaceGlRef _params;

    bool _visible = true;
};