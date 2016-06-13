#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/Camera.h"
#include "cinder/params/Params.h"
#include "cinder/gl/gl.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
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

    float _point_size;
    bool _visible_grid;

    float _voxel_size;
    bool _enabled_voxel_filter;

    int _sor_meank;
    float _sor_std_dev_mul_th;
    bool _enabled_sor;

    void updatePointCloud();

    const string OPT_CAMERA = "Camera";
    const string OPT_VOXEL = "Voxel Filter";
    const string OPT_SOR = "Statistical Outlier Removal";
};

void CiPcdViewerApp::setup()
{
    _point_size = 0.005f;
    _visible_grid = true;

    _camera_target = vec3(0);
    _camera_eye_point = vec3(3);

    _voxel_size = 0.05f;
    _enabled_voxel_filter = false;
    _sor_meank = 50;
    _sor_std_dev_mul_th = 1.0f;
    _enabled_sor = false;

    _params = params::InterfaceGl::create(getWindow(), "CiPcdViewer", toPixels(ivec2(200, 400)));

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
}

void CiPcdViewerApp::updatePointCloud() {
    for (auto cloud_gl : _clouds) {
        cloud_gl.second->filter([this](pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);

            if (_enabled_voxel_filter) {
                pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
                sor.setInputCloud(cloud);
                sor.setLeafSize(_voxel_size, _voxel_size, _voxel_size);
                sor.filter(*cloud_tmp);
            } else {
                pcl::copyPointCloud(*cloud, *cloud_tmp);
            }

            if (_enabled_sor) {
                pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
                sor.setInputCloud(cloud_tmp);
                sor.setMeanK(_sor_meank);
                sor.setStddevMulThresh(_sor_std_dev_mul_th);
                sor.filter(*cloud);
            } else {
                pcl::copyPointCloud(*cloud_tmp, *cloud);
            }
        });

        cloud_gl.second->update();
    }
}

void CiPcdViewerApp::mouseDown( MouseEvent event )
{
}

void CiPcdViewerApp::update()
{
    stringstream ss;
    ss << "label=`FPS: " << getAverageFps() << "`";
    _params->setOptions("fps", ss.str());

    gl::enableDepthRead();
    gl::enableDepthWrite();
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
        for (int i = -5; i < 5; i++) {
            for (int j = -5; j < 5; j++) {
                gl::drawStrokedRect(Rectf(i, j, i + 1, j + 1));
            }
        }
        gl::popMatrices();
    }

    for (auto cloud_gl : _clouds) {
        cloud_gl.second->draw();
    }
    
    _params->draw();
}

CINDER_APP( CiPcdViewerApp, RendererGl, [](App::Settings *settings) {
    settings->setHighDensityDisplayEnabled();
})
