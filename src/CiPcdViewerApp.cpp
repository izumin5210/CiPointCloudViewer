#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/Camera.h"
#include "cinder/params/Params.h"
#include "cinder/gl/gl.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <map>

#include "CloudGl.hpp"

using namespace ci;
using namespace ci::app;
using namespace std;

class CiPcdViewerApp : public App {
public:
    void setup() override;
    void mouseDown( MouseEvent event ) override;
    void update() override;
	void draw() override;

private:
    params::InterfaceGlRef _params;

    CameraPersp _camera;
    vec3 _camera_target;
    vec3 _camera_eye_point;

    map<fs::path, shared_ptr<CloudGl>> _clouds;

    gl::VertBatchRef _batch;

    float _point_size;
    bool _visible_grid;

    float _voxel_size;
    bool _enabled_voxel_filter;

    bool _enable_pass_through_x = false;
    float _min_pass_through_x = -1.0f;
    float _max_pass_through_x = 1.0f;
    bool _enable_pass_through_y = false;
    float _min_pass_through_y = -1.0f;
    float _max_pass_through_y = 1.0f;
    bool _enable_pass_through_z = false;
    float _min_pass_through_z = -1.0f;
    float _max_pass_through_z = 1.0f;
                             
    int _sor_meank;
    float _sor_std_dev_mul_th;
    bool _enabled_sor;

    void updatePointCloud();

    const string OPT_CAMERA = "Camera";
    const string OPT_VOXEL = "Voxel Filter";
    const string OPT_SOR = "Statistical Outlier Removal";
    const string OPT_PASS_THROUGH = "PassThrough Filter";
};

void CiPcdViewerApp::setup()
{
    _batch = gl::VertBatch::create(GL_POINTS);

    _point_size = 0.005f;
    _visible_grid = true;

    _camera_target = vec3(0, 0.5, 0);
    _camera_eye_point = vec3(4, 2, -4);

    _voxel_size = 0.05f;
    _enabled_voxel_filter = false;
    _sor_meank = 50;
    _sor_std_dev_mul_th = 1.0f;
    _enabled_sor = false;

    _params = params::InterfaceGl::create(getWindow(), "CiPcdViewer", toPixels(ivec2(200, 640)));

    _params->addButton("Open *.pcd file", [this]() {
        auto pcdfile = getOpenFilePath();
        _clouds[pcdfile] = std::make_shared<CloudGl>(pcdfile);
        updatePointCloud();
    });

    _params->addParam("Grid", &_visible_grid);

    _params->addParam("Point Size", &_point_size)
        .min(1.0f)
        .max(10.0f)
        .step(0.1f);

    _params->addParam("Look At", &_camera_target)
        .group(OPT_CAMERA);

    _params->addParam("Eye Point", &_camera_eye_point)
        .group(OPT_CAMERA);

    _params->addSeparator();

    _params->addParam("X Filter", &_enable_pass_through_x)
        .group(OPT_PASS_THROUGH)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    _params->addParam("Min X", &_min_pass_through_x)
        .min(-5.0f)
        .max(5.0f)
        .step(0.02f)
        .group(OPT_PASS_THROUGH)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    _params->addParam("Max X", &_max_pass_through_x)
        .min(-5.0f)
        .max(5.0f)
        .step(0.02f)
        .group(OPT_PASS_THROUGH)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    _params->addParam("Y Filter", &_enable_pass_through_y)
        .group(OPT_PASS_THROUGH)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    _params->addParam("Min Y", &_min_pass_through_y)
        .min(-5.0f)
        .max(5.0f)
        .step(0.02f)
        .group(OPT_PASS_THROUGH)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    _params->addParam("Max Y", &_max_pass_through_y)
        .min(-5.0f)
        .max(5.0f)
        .step(0.02f)
        .group(OPT_PASS_THROUGH)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    _params->addParam("Z Filter", &_enable_pass_through_z)
        .group(OPT_PASS_THROUGH)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    _params->addParam("Min Z", &_min_pass_through_z)
        .min(-5.0f)
        .max(5.0f)
        .step(0.02f)
        .group(OPT_PASS_THROUGH)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    _params->addParam("Max Z", &_max_pass_through_z)
        .min(-5.0f)
        .max(5.0f)
        .step(0.02f)
        .group(OPT_PASS_THROUGH)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    _params->addParam("Enable Voxel Filter", &_enabled_voxel_filter)
        .group(OPT_VOXEL)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    _params->addParam("Voxel Size", &_voxel_size)
        .min(0.001f)
        .max(1.0f)
        .step(0.001f)
        .group(OPT_VOXEL)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    _params->addParam("Enable SOR", &_enabled_sor)
        .group(OPT_SOR)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    _params->addParam("MeanK", &_sor_meank)
        .min(1)
        .max(100)
        .step(1)
        .group(OPT_SOR)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    _params->addParam("StddevMulThresh", &_sor_std_dev_mul_th)
        .min(0.0f)
        .max(10.0f)
        .step(0.1f)
        .group(OPT_SOR)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    _params->addSeparator();
    _params->addText("fps", "label=`FPS: `");
    _params->addText("cloud_size", "label=`Cloud Size: `");
    _params->addText("filtered_cloud_size", "label=`Filtered: `");

    gl::enableDepthRead();
    gl::enableDepthWrite();
}

void CiPcdViewerApp::updatePointCloud() {
    _batch->clear();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);

    for (auto cloud_gl : _clouds) {
        *cloud += *(cloud_gl.second->cloud());
    }

    std::stringstream ss_input;
    ss_input << "label=`Cloud Size: " << cloud->size() << "`";
    _params->setOptions("cloud_size", ss_input.str());

    if (_enable_pass_through_x) {
        pcl::PassThrough<pcl::PointXYZRGBA> pass_x;
        pass_x.setInputCloud(cloud);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(_min_pass_through_x, _max_pass_through_x);
        pass_x.filter(*cloud_tmp);
    } else {
        pcl::copyPointCloud(*cloud, *cloud_tmp);
    }

    if (_enable_pass_through_y) {
        pcl::PassThrough<pcl::PointXYZRGBA> pass_y;
        pass_y.setInputCloud(cloud_tmp);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(_min_pass_through_y, _max_pass_through_y);
        pass_y.filter(*cloud);
    } else {
        pcl::copyPointCloud(*cloud_tmp, *cloud);
    }

    if (_enable_pass_through_z) {
        pcl::PassThrough<pcl::PointXYZRGBA> pass_z;
        pass_z.setInputCloud(cloud);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(_min_pass_through_z, _max_pass_through_z);
        pass_z.filter(*cloud_tmp);
    } else {
        pcl::copyPointCloud(*cloud, *cloud_tmp);
    }

    if (_enabled_voxel_filter) {
        pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
        sor.setInputCloud(cloud_tmp);
        sor.setLeafSize(_voxel_size, _voxel_size, _voxel_size);
        sor.filter(*cloud);
    } else {
        pcl::copyPointCloud(*cloud_tmp, *cloud);
    }

    if (_enabled_sor) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(_sor_meank);
        sor.setStddevMulThresh(_sor_std_dev_mul_th);
        sor.filter(*cloud_tmp);
        pcl::copyPointCloud(*cloud_tmp, *cloud);
    }

    for (auto point : cloud->points) {
        _batch->color(ci::ColorA8u(point.r, point.g, point.b, point.a));
        _batch->vertex(ci::vec3(point.x, point.y, point.z));
    }

    std::stringstream ss_filtered;
    ss_filtered << "label=`Filtered: " << cloud->size() << "`";
    _params->setOptions("filtered_cloud_size", ss_filtered.str());
}

void CiPcdViewerApp::mouseDown( MouseEvent event )
{
}

void CiPcdViewerApp::update()
{
    stringstream ss;
    ss << "label=`FPS: " << getAverageFps() << "`";
    _params->setOptions("fps", ss.str());
}

void CiPcdViewerApp::draw()
{
    gl::clear();

    _camera.lookAt(_camera_eye_point, _camera_target);
    gl::setMatrices(_camera);

    gl::pointSize(_point_size);

    if (_visible_grid) {
        gl::pushMatrices();
        gl::color(1, 1, 1, 0.3);
        gl::rotate(M_PI_2, vec3(1, 0, 0));
        for (float i = -5; i < 5; i += 0.5) {
            for (float j = -5; j < 5; j += 0.5) {
                gl::drawStrokedRect(Rectf(i, j, i + 0.5, j + 0.5));
            }
        }
        gl::popMatrices();
    }

    _batch->draw();

    _params->draw();
}

CINDER_APP( CiPcdViewerApp, RendererGl, [](App::Settings *settings) {
    settings->setHighDensityDisplayEnabled();
    settings->setWindowSize(1280, 960);
    settings->setFrameRate(240.0f);
})
