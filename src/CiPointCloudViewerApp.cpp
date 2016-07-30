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

#include "AppGui.h"
#include "Configure.h"
#include "io/CloudDataSources.h"

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

  AppGui gui_;

  gl::VertBatchRef grid_batch_;

  gl::GlslProgRef render_prog_;
  map<Clouds::Key, gl::VaoRef> vaos_;
  map<Clouds::Key, gl::VboRef> vbos_;

  CameraPersp camera_;
  CameraUi camera_ui_;

  std::atomic<bool> cloud_updated_;

  void onCloudsUpdate();
  void onViewParamsUpdate();
  void updateVbo();
};

CiPointCloudViewerApp::CiPointCloudViewerApp()
  : clouds_(new Clouds)
  , view_params_(new ViewParams)
  , config_(new Configure(getAssetPath("")))
  , sensor_device_manager_(new io::SensorDeviceManager)
  , cloud_data_sources_(new io::CloudDataSources)
  , gui_(clouds_, view_params_, config_, cloud_data_sources_, sensor_device_manager_)
  , grid_batch_(gl::VertBatch::create(GL_LINES))
  , render_prog_(
    gl::GlslProg::create(
      gl::GlslProg::Format()
        .vertex(loadAsset("pointcloud.vert"))
        .fragment(loadAsset("pointcloud.frag"))
    )
  )
  , camera_ui_(&camera_)
  , cloud_updated_(false)
{}

CiPointCloudViewerApp::~CiPointCloudViewerApp() {
  sensor_device_manager_->stop();
}

void CiPointCloudViewerApp::setup() {
  clouds_->connect(this, &CiPointCloudViewerApp::onCloudsUpdate);
  view_params_->connect(this, &CiPointCloudViewerApp::onViewParamsUpdate);

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

  gl::enableFaceCulling(true);
  gl::enableVerticalSync(false);
  disableFrameRate();
  gl::enableDepthRead();
  gl::enableDepthWrite();

  onViewParamsUpdate();
}

void CiPointCloudViewerApp::onCloudsUpdate() {
  cloud_updated_ = false;
}

void CiPointCloudViewerApp::onViewParamsUpdate() {
  camera_.setEyePoint(view_params_->eye_point());
  camera_.lookAt(view_params_->eye_point(), view_params_->look_at());
}


void CiPointCloudViewerApp::updateVbo() {
  cloud_updated_ = true;
  clouds_->lock();
  for (auto pair : clouds_->clouds()) {
    if (pair.second.empty()) { continue; }
    if (vaos_.find(pair.first) == vaos_.end()) {
      vaos_[pair.first] = gl::Vao::create();
    }
    if (vbos_.find(pair.first) == vbos_.end()) {
      vbos_[pair.first] = gl::Vbo::create(GL_ARRAY_BUFFER, 0, nullptr, GL_STATIC_DRAW);
    }
    vbos_[pair.first]->copyData(pair.second.size() * sizeof(Point), pair.second.data());
    gl::ScopedVao vao(vaos_[pair.first]);
    gl::ScopedBuffer vbo(vbos_[pair.first]);
    gl::enableVertexAttribArray(0);
    gl::enableVertexAttribArray(1);
    gl::vertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Point), (const GLvoid*)offsetof(Point, xyz));
    gl::vertexAttribPointer(1, 3, GL_UNSIGNED_BYTE, GL_FALSE, sizeof(Point), (const GLvoid*)offsetof(Point, rgb));
  }
  clouds_->unlock();
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
  gui_.update(this);

  if (!cloud_updated_) {
    updateVbo();
  }

  setFullScreen(view_params_->is_full_screen());
}

void CiPointCloudViewerApp::draw() {
  gl::clear(view_params_->bg_color());

  gl::setMatrices(camera_);

  gl::pointSize(view_params_->point_size());

  if (view_params_->is_visible_grid()) {
      grid_batch_->draw();
  }

  {
    gl::ScopedGlslProg render(render_prog_);
    clouds_->lock();
    for (auto pair : vaos_) {
      auto calib_params = clouds_->calib_params_map()[pair.first];
      mat4 calib_matrix(1.0f);
      for (long i = 0; i < calib_params.calib_matrix.cols(); i++) {
        for (long j = 0; j < calib_params.calib_matrix.rows(); j++) {
          calib_matrix[i][j] = calib_params.calib_matrix(j, i);
        }
      }
      render_prog_->uniform("calibMatrix", calib_matrix);
      render_prog_->uniform("fx", calib_params.fx);
      render_prog_->uniform("fy", calib_params.fy);
      render_prog_->uniform("cx", calib_params.cx);
      render_prog_->uniform("cy", calib_params.cy);
      gl::ScopedVao vao(pair.second);
      gl::context()->setDefaultShaderVars();
      gl::drawArrays(GL_POINTS, 0, clouds_->clouds()[pair.first].size());
    }
    clouds_->unlock();
  }
}

CINDER_APP( CiPointCloudViewerApp, RendererGl, [](App::Settings *settings) {
  settings->setHighDensityDisplayEnabled();
  settings->setWindowSize(1280, 960);
})

