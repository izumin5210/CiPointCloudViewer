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
    const string kOptCamera = "Camera";
    const string kOptVoxel = "Voxel Filter";
    const string kOptSor = "Statistical Outlier Removal";
    const string kOptPassThrough = "PassThrough Filter";

    params::InterfaceGlRef params_;

    CameraPersp camera_;
    vec3 camera_target_;
    vec3 camera_eye_point_;

    map<fs::path, shared_ptr<CloudGl>> clouds_;

    gl::VertBatchRef batch_;

    float point_size_;
    bool visible_grid_;

    float voxel_size_;
    bool enabled_voxel_filter_;

    bool enable_pass_through_x_ = false;
    float min_pass_through_x_ = -1.0f;
    float max_pass_through_x_ = 1.0f;
    bool enable_pass_through_y_ = false;
    float min_pass_through_y_ = -1.0f;
    float max_pass_through_y_ = 1.0f;
    bool enable_pass_through_z_ = false;
    float min_pass_through_z_ = -1.0f;
    float max_pass_through_z_ = 1.0f;

    int sor_meank_;
    float sor_std_dev_mul_th_;
    bool enabled_sor_;

    void updatePointCloud();
};

void CiPcdViewerApp::setup()
{
    batch_ = gl::VertBatch::create(GL_POINTS);

    point_size_ = 1.0f;
    visible_grid_ = true;

    camera_target_ = vec3(0, 0.5, 0);
    camera_eye_point_ = vec3(4, 2, -4);

    voxel_size_ = 0.05f;
    enabled_voxel_filter_ = false;
    sor_meank_ = 50;
    sor_std_dev_mul_th_ = 1.0f;
    enabled_sor_ = false;

    params_ = params::InterfaceGl::create(getWindow(), "CiPcdViewer", toPixels(ivec2(200, 640)));

    params_->addButton("Open *.pcd file", [this]() {
        auto pcdfile = getOpenFilePath();
        clouds_[pcdfile] = std::make_shared<CloudGl>(pcdfile);
        updatePointCloud();
    });

    params_->addParam("Grid", &visible_grid_);

    params_->addParam("Point Size", &point_size_)
        .min(1.0f)
        .max(10.0f)
        .step(0.1f);

    params_->addParam("Look At", &camera_target_)
        .group(kOptCamera);

    params_->addParam("Eye Point", &camera_eye_point_)
        .group(kOptCamera);

    params_->addSeparator();

    params_->addParam("X Filter", &enable_pass_through_x_)
        .group(kOptPassThrough)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    params_->addParam("Min X", &min_pass_through_x_)
        .min(-5.0f)
        .max(5.0f)
        .step(0.02f)
        .group(kOptPassThrough)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    params_->addParam("Max X", &max_pass_through_x_)
        .min(-5.0f)
        .max(5.0f)
        .step(0.02f)
        .group(kOptPassThrough)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    params_->addParam("Y Filter", &enable_pass_through_y_)
        .group(kOptPassThrough)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    params_->addParam("Min Y", &min_pass_through_y_)
        .min(-5.0f)
        .max(5.0f)
        .step(0.02f)
        .group(kOptPassThrough)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    params_->addParam("Max Y", &max_pass_through_y_)
        .min(-5.0f)
        .max(5.0f)
        .step(0.02f)
        .group(kOptPassThrough)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    params_->addParam("Z Filter", &enable_pass_through_z_)
        .group(kOptPassThrough)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    params_->addParam("Min Z", &min_pass_through_z_)
        .min(-5.0f)
        .max(5.0f)
        .step(0.02f)
        .group(kOptPassThrough)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    params_->addParam("Max Z", &max_pass_through_z_)
        .min(-5.0f)
        .max(5.0f)
        .step(0.02f)
        .group(kOptPassThrough)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    params_->addParam("Enable Voxel Filter", &enabled_voxel_filter_)
        .group(kOptVoxel)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    params_->addParam("Voxel Size", &voxel_size_)
        .min(0.001f)
        .max(1.0f)
        .step(0.001f)
        .group(kOptVoxel)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    params_->addParam("Enable SOR", &enabled_sor_)
        .group(kOptSor)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    params_->addParam("MeanK", &sor_meank_)
        .min(1)
        .max(100)
        .step(1)
        .group(kOptSor)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    params_->addParam("StddevMulThresh", &sor_std_dev_mul_th_)
        .min(0.0f)
        .max(10.0f)
        .step(0.1f)
        .group(kOptSor)
        .updateFn(bind(&CiPcdViewerApp::updatePointCloud, this));

    params_->addSeparator();
    params_->addText("fps", "label=`FPS: `");
    params_->addText("cloud_size", "label=`Cloud Size: `");
    params_->addText("filtered_cloud_size", "label=`Filtered: `");

    gl::enableDepthRead();
    gl::enableDepthWrite();
}

void CiPcdViewerApp::updatePointCloud() {
    batch_->clear();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);

    for (auto cloud_gl : clouds_) {
        *cloud += *(cloud_gl.second->cloud());
    }

    std::stringstream ss_input;
    ss_input << "label=`Cloud Size: " << cloud->size() << "`";
    params_->setOptions("cloud_size", ss_input.str());

    if (enable_pass_through_x_) {
        pcl::PassThrough<pcl::PointXYZRGBA> pass_x;
        pass_x.setInputCloud(cloud);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(min_pass_through_x_, max_pass_through_x_);
        pass_x.filter(*cloud_tmp);
    } else {
        pcl::copyPointCloud(*cloud, *cloud_tmp);
    }

    if (enable_pass_through_y_) {
        pcl::PassThrough<pcl::PointXYZRGBA> pass_y;
        pass_y.setInputCloud(cloud_tmp);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(min_pass_through_y_, max_pass_through_y_);
        pass_y.filter(*cloud);
    } else {
        pcl::copyPointCloud(*cloud_tmp, *cloud);
    }

    if (enable_pass_through_z_) {
        pcl::PassThrough<pcl::PointXYZRGBA> pass_z;
        pass_z.setInputCloud(cloud);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(min_pass_through_z_, max_pass_through_z_);
        pass_z.filter(*cloud_tmp);
    } else {
        pcl::copyPointCloud(*cloud, *cloud_tmp);
    }

    if (enabled_voxel_filter_) {
        pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
        sor.setInputCloud(cloud_tmp);
        sor.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        sor.filter(*cloud);
    } else {
        pcl::copyPointCloud(*cloud_tmp, *cloud);
    }

    if (enabled_sor_) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(sor_meank_);
        sor.setStddevMulThresh(sor_std_dev_mul_th_);
        sor.filter(*cloud_tmp);
        pcl::copyPointCloud(*cloud_tmp, *cloud);
    }

    for (auto point : cloud->points) {
        batch_->color(ci::ColorA8u(point.r, point.g, point.b, point.a));
        batch_->vertex(ci::vec3(point.x, point.y, point.z));
    }

    std::stringstream ss_filtered;
    ss_filtered << "label=`Filtered: " << cloud->size() << "`";
    params_->setOptions("filtered_cloud_size", ss_filtered.str());
}

void CiPcdViewerApp::mouseDown( MouseEvent event )
{
}

void CiPcdViewerApp::update()
{
    stringstream ss;
    ss << "label=`FPS: " << getAverageFps() << "`";
    params_->setOptions("fps", ss.str());
}

void CiPcdViewerApp::draw()
{
    gl::clear();

    camera_.lookAt(camera_eye_point_, camera_target_);
    gl::setMatrices(camera_);

    gl::pointSize(point_size_);

    if (visible_grid_) {
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

    batch_->draw();

    params_->draw();
}

CINDER_APP( CiPcdViewerApp, RendererGl, [](App::Settings *settings) {
    settings->setHighDensityDisplayEnabled();
    settings->setWindowSize(1280, 960);
    settings->setFrameRate(240.0f);
})
