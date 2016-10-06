#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/Camera.h"
#include "cinder/CameraUi.h"
#include "cinder/gl/gl.h"
#include "cinder/Signals.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <set>
#include <map>
#include <Signal.h>

#include "CinderImGui.h"

#include "view/AppGui.h"
#include "Configure.h"
#include "io/CloudDataSources.h"
#include "SavingVerticesWorker.h"

#include "renderer/CloudsRenderer.h"
#include "renderer/GridRenderer.h"

#include "Clouds.h"
#include "ViewParams.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class CiPointCloudViewerApp : public App {
public:
  using PointT        = pcl::PointXYZRGBA;
  using PointCloud    = pcl::PointCloud<PointT>;
  using PointCloudPtr = PointCloud::Ptr;

  CiPointCloudViewerApp();
  ~CiPointCloudViewerApp();

  void setup() override;
  void mouseDown(MouseEvent event) override;
  void mouseDrag(MouseEvent event) override;
  void mouseWheel(MouseEvent event) override;
  void update() override;
  void draw() override;

private:
  shared_ptr<Clouds> clouds_;
  shared_ptr<ViewParams> view_params_;
  shared_ptr<Configure> config_;
  shared_ptr<io::SensorDeviceManager> sensor_device_manager_;
  shared_ptr<io::CloudDataSources> cloud_data_sources_;
  shared_ptr<SavingVerticesWorker> saving_vertices_worker_;

  view::AppGui gui_;
  renderer::CloudsRenderer clouds_renderer_;
  renderer::GridRenderer grid_renderer_;

  CameraPersp camera_;
  CameraUi camera_ui_;

  void onViewParamsUpdate();
};

CiPointCloudViewerApp::CiPointCloudViewerApp()
  : clouds_(new Clouds)
  , view_params_(new ViewParams)
  , config_(new Configure(getAssetPath("")))
  , sensor_device_manager_(new io::SensorDeviceManager)
  , cloud_data_sources_(new io::CloudDataSources)
  , saving_vertices_worker_(new SavingVerticesWorker)
  , gui_(this, clouds_, view_params_, config_, cloud_data_sources_, sensor_device_manager_, saving_vertices_worker_)
  , clouds_renderer_(this, clouds_)
  , grid_renderer_(view_params_)
  , camera_ui_(&camera_)
{}

CiPointCloudViewerApp::~CiPointCloudViewerApp() {
  sensor_device_manager_->stop();
}

void CiPointCloudViewerApp::setup() {
  view_params_->connect(std::bind(&CiPointCloudViewerApp::onViewParamsUpdate, this));

  config_->initialize();
  gui_.initialize();

  sensor_device_manager_->start();

  gl::enableFaceCulling(true);
  gl::enableVerticalSync(false);
  disableFrameRate();
  gl::enableDepthRead();
  gl::enableDepthWrite();

  onViewParamsUpdate();
}

void CiPointCloudViewerApp::onViewParamsUpdate() {
  camera_.setEyePoint(view_params_->eye_point());
  camera_.lookAt(view_params_->eye_point(), view_params_->look_at());
}


void CiPointCloudViewerApp::mouseDown(MouseEvent event) {
  camera_ui_.mouseDown(event);
  Signal<ViewParams::UpdateCameraParamsAction>::emit({camera_.getEyePoint(), camera_.getPivotPoint()});
}

void CiPointCloudViewerApp::mouseDrag(MouseEvent event) {
  camera_ui_.mouseDrag(event);
  Signal<ViewParams::UpdateCameraParamsAction>::emit({camera_.getEyePoint(), camera_.getPivotPoint()});
}

void CiPointCloudViewerApp::mouseWheel(MouseEvent event) {
  camera_ui_.mouseWheel(event);
  Signal<ViewParams::UpdateCameraParamsAction>::emit({camera_.getEyePoint(), camera_.getPivotPoint()});
}

void CiPointCloudViewerApp::update() {
  gui_.update();
  grid_renderer_.update();
  clouds_renderer_.update();

  setFullScreen(view_params_->is_full_screen());
}

void CiPointCloudViewerApp::draw() {
  gl::clear(view_params_->bg_color());

  gl::setMatrices(camera_);

  gl::pointSize(view_params_->point_size());

  grid_renderer_.render();
  clouds_renderer_.render();
}

CINDER_APP( CiPointCloudViewerApp, RendererGl, [](App::Settings *settings) {
  settings->setHighDensityDisplayEnabled();
  settings->setWindowSize(1280, 960);
})

