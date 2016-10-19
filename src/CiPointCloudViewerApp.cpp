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

#include "renderer/cloud/CloudsRenderer.h"
#include "renderer/grid/GridRenderer.h"

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
  renderer::cloud::CloudsRenderer clouds_renderer_;
  renderer::grid::GridRenderer grid_renderer_;

  CameraUi camera_ui_;
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
  , camera_ui_(&(view_params_->camera()))
{}

CiPointCloudViewerApp::~CiPointCloudViewerApp() {
  sensor_device_manager_->stop();
  camera_ui_.disconnect();
}

void CiPointCloudViewerApp::setup() {
  config_->initialize();
  gui_.initialize();

  sensor_device_manager_->start();
  camera_ui_.connect(getWindow());

  gl::enableFaceCulling(true);
  gl::enableVerticalSync(false);
  disableFrameRate();
  gl::enableDepthRead();
  gl::enableDepthWrite();
}

void CiPointCloudViewerApp::update() {
  gui_.update();
  grid_renderer_.update();
  clouds_renderer_.update();

  setFullScreen(view_params_->is_full_screen());
}

void CiPointCloudViewerApp::draw() {
  gl::clear(view_params_->bg_color());
  gl::setMatrices(view_params_->camera());
  gl::pointSize(view_params_->point_size());

  grid_renderer_.render();
  clouds_renderer_.render();
}

CINDER_APP( CiPointCloudViewerApp, RendererGl, [](App::Settings *settings) {
  settings->setHighDensityDisplayEnabled();
  settings->setWindowSize(1280, 960);
})

