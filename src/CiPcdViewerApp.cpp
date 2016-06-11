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

#include <sstream>

using namespace ci;
using namespace ci::app;
using namespace std;

class CiPcdViewerApp : public App {
public:
    CiPcdViewerApp()
        : _cloud(new pcl::PointCloud<pcl::PointXYZRGBA>)
        , _cloud_input(new pcl::PointCloud<pcl::PointXYZRGBA>)
        , _cloud_batch(gl::VertBatch::create(GL_POINTS))
    {
    }
    
    void setup() override;
    void mouseDown( MouseEvent event ) override;
    void update() override;
	void draw() override;
    
private:
    params::InterfaceGlRef _params;
    
    CameraPersp _camera;
    
    fs::path _pcdfile;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _cloud_input;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _cloud;
    
    gl::VertBatchRef _cloud_batch;
    
    float _point_size;
    float _voxel_size;
    bool _enabled_voxel_filter;
    
    int _sor_meank;
    float _sor_std_dev_mul_th;
    bool _enabled_sor;
    
    void updatePointCloud();
    
    const string OPT_VOXEL = "Voxel Filter";
    const string OPT_SOR = "Statistical Outlier Removal";
};

void CiPcdViewerApp::setup()
{
    _point_size = 0.005f;
    _voxel_size = 0.05f;
    _enabled_voxel_filter = false;
    _sor_meank = 50;
    _sor_std_dev_mul_th = 1.0f;
    _enabled_sor = false;
    
    _params = params::InterfaceGl::create(getWindow(), "CiPcdViewer", toPixels(ivec2(200, 300)));
    
    _params->addButton("Open *.pcd file", [this]() {
        _pcdfile = getOpenFilePath();
        cout << _pcdfile << endl;
        pcl::io::loadPCDFile(_pcdfile.string(), *_cloud_input);
        updatePointCloud();
    });
    
    _params->addParam("Point Size", &_point_size)
        .min(1.0f)
        .max(10.0f)
        .step(0.1f);
    
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
    _params->addText("cloud_size", "label=`Cloud Size: 0`");
    _params->addText("filtered_cloud_size", "label=`Filtered: 0`");
    _params->addSeparator();
    _params->addText("fps", "label=`FPS: `");
}

void CiPcdViewerApp::updatePointCloud() {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (_enabled_voxel_filter) {                      
        pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
        sor.setInputCloud(_cloud_input);
        sor.setLeafSize(_voxel_size, _voxel_size, _voxel_size);
        sor.filter(*cloud_tmp);
    } else {
        pcl::copyPointCloud(*_cloud_input, *cloud_tmp);
    }
    
    if (_enabled_sor) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
        sor.setInputCloud(cloud_tmp);
        sor.setMeanK(_sor_meank);
        sor.setStddevMulThresh(_sor_std_dev_mul_th);
        sor.filter(*_cloud);
    } else {
        pcl::copyPointCloud(*cloud_tmp, *_cloud);
    }
    
    stringstream ss_input;
    ss_input << "label=`Cloud Size: " << _cloud_input->size() << "`";
    _params->setOptions("cloud_size", ss_input.str());
    stringstream ss_filtered;
    ss_filtered << "label=`Filtered: " << _cloud->size() << "`";
    _params->setOptions("filtered_cloud_size", ss_filtered.str());
    
    _cloud_batch->clear();
    for (auto point : _cloud->points) {
        _cloud_batch->color(ColorA8u(point.r, point.g, point.b, point.a));
        _cloud_batch->vertex(vec3(point.x, point.y, point.z));
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
    
    _camera.lookAt(vec3(3, 3, 3), vec3(0, 0, 0));
    gl::setMatrices(_camera);
    
    gl::pointSize(_point_size);
    _cloud_batch->draw();
    
    _params->draw();
}

CINDER_APP( CiPcdViewerApp, RendererGl, [](App::Settings *settings) {
    settings->setHighDensityDisplayEnabled();
})
