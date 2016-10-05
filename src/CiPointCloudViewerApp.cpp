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

  gl::VertBatchRef grid_batch_;
  gl::VertBatchRef circular_grid_batches_[6];

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
  , grid_batch_(gl::VertBatch::create(GL_LINES))
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

  grid_batch_->color(1, 1, 1, 0.3);
  for (float i = -5; i <= 5.0; i += 0.5) {
    for (float j = -5; j <= 5.0; j += 0.5) {
      grid_batch_->vertex(vec3( i, 0, -j));
      grid_batch_->vertex(vec3( i, 0,  j));
      grid_batch_->vertex(vec3(-i, 0,  j));
      grid_batch_->vertex(vec3( i, 0,  j));
    }
  }
  {
    for (int i = 0; i < 5; i++) {
      auto batch = gl::VertBatch::create(GL_LINE_LOOP);
      batch->color(1, 1, 1, 0.3);
      for (int j = 0; j < 360; j++) {
        float rad = (float) (j * M_PI) / 180;
        batch->vertex((i + 1) * cos(rad), 0, (i + 1) * sin(rad));
      }
      circular_grid_batches_[i] = batch;
    }
    auto batch = gl::VertBatch::create(GL_LINES);
    batch->color(1, 1, 1, 0.3);
    for (int i = 0; i < 6; i++) {
      auto x = 5 * cos(M_PI / 6 * i);
      auto z = 5 * sin(M_PI / 6 * i);
      batch->vertex(vec3(x, 0, z));
      batch->vertex(vec3(-x, 0, -z));
    }
    circular_grid_batches_[5] = batch;
  }


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
  clouds_renderer_.update();

  setFullScreen(view_params_->is_full_screen());
}

void CiPointCloudViewerApp::draw() {
  gl::clear(view_params_->bg_color());

  gl::setMatrices(camera_);

  gl::pointSize(view_params_->point_size());

  switch (view_params_->grid()) {
    case ViewParams::Grid::RECTANGULAR:
      grid_batch_->draw();
      break;
    case ViewParams::Grid::POLAR:
      for (auto batch : circular_grid_batches_) {
        batch->draw();
      }
      break;
    case ViewParams::Grid::NONE:
      break;
  }

  clouds_renderer_.render();
}

CINDER_APP( CiPointCloudViewerApp, RendererGl, [](App::Settings *settings) {
  settings->setHighDensityDisplayEnabled();
  settings->setWindowSize(1280, 960);
})

