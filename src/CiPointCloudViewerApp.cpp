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

#include "CinderImGui.h"


#include "action/ViewParamsAction.h"

#include "impl/AppGuiImpl.h"
#include "impl/CloudsImpl.h"
#include "impl/ViewParamsImpl.h"
#include "impl/SavingVerticesWorkerImpl.h"
#include "impl/YamlConfigure.h"
#include "impl/io/CloudDataSourcesImpl.h"
#include "impl/io/SensorDeviceManagerImpl.h"

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
  shared_ptr<Dispatcher> dispatcher_;
  shared_ptr<Clouds> clouds_;
  shared_ptr<ViewParams> view_params_;
  shared_ptr<io::SensorDeviceManager> sensor_device_manager_;

  shared_ptr<AppGui> gui_;

  gl::VertBatchRef grid_batch_;
  gl::VertBatchRef circular_grid_batches_[6];

  gl::GlslProgRef vertices_render_prog_;
  map<Cloud::Key, gl::VaoRef> vertices_vaos_;
  map<Cloud::Key, gl::VboRef> vertices_vbos_;

  gl::GlslProgRef points_render_prog_;
  map<Cloud::Key, gl::VaoRef> points_vaos_;
  map<Cloud::Key, gl::VboRef> points_vbos_;

  size_t vertices_size_;
  size_t points_size_;

  CameraPersp camera_;
  CameraUi camera_ui_;

  std::atomic<bool> cloud_updated_;

  void onCloudsUpdate();
  void onViewParamsUpdate();
  void updateVerticesVbo();
  void updatePointsVbo();
};

CiPointCloudViewerApp::CiPointCloudViewerApp()
  : grid_batch_(gl::VertBatch::create(GL_LINES))
  , vertices_render_prog_(
    gl::GlslProg::create(
      gl::GlslProg::Format()
        .vertex(loadAsset("vertices.vert"))
        .fragment(loadAsset("point_cloud.frag"))
    )
  )
  , points_render_prog_(
    gl::GlslProg::create(
      gl::GlslProg::Format()
        .vertex(loadAsset("point_cloud.vert"))
        .fragment(loadAsset("point_cloud.frag"))
    )
  )
  , vertices_size_(0)
  , points_size_(0)
  , camera_ui_(&camera_)
  , cloud_updated_(false)
{}

CiPointCloudViewerApp::~CiPointCloudViewerApp() {
  sensor_device_manager_->stop();
}

void CiPointCloudViewerApp::setup() {
  cout << "setup App" << endl;
  cout << "begin: setup components" << endl;
  fruit::Component<AppBase> app_component =
      fruit::createComponent().bindInstance(*this->get());
  fruit::Component<
      Dispatcher,
      Clouds,
      ViewParams,
      AppGui,
      io::SensorDeviceManager
  > component = fruit::createComponent()
      .install(app_component)
      .install(getDispatcherComponent())
      .install(getCloudsImplComponent())
      .install(getViewParamsImplComponent())
      .install(getYamlConfigureComponent())
      .install(getSensorDeviceManagerImplComponent())
      .install(getCloudDataSourcesImplComponent())
      .install(getSavingVerticesWorkerImplComponent())
      .install(getAppGuiImplComponent());
  fruit::Injector<
      Dispatcher,
      Clouds,
      ViewParams,
      AppGui,
      io::SensorDeviceManager
  > injector(component);
  cout << "end:   setup components" << endl;
  cout << "begin: injection to App" << endl;

  dispatcher_ = injector.get<shared_ptr<Dispatcher>>();
  clouds_ = injector.get<shared_ptr<Clouds>>();
  view_params_ = injector.get<shared_ptr<ViewParams>>();
  gui_ = injector.get<shared_ptr<AppGui>>();
  sensor_device_manager_ = injector.get<shared_ptr<io::SensorDeviceManager>>();

  clouds_->connect(std::bind(&CiPointCloudViewerApp::onCloudsUpdate, this));
  view_params_->connect(std::bind(&CiPointCloudViewerApp::onViewParamsUpdate, this));
  cout << "end:   injection to App" << endl;

  gui_->initialize();

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

void CiPointCloudViewerApp::onCloudsUpdate() {
  cloud_updated_ = false;
}

void CiPointCloudViewerApp::onViewParamsUpdate() {
  camera_.setEyePoint(view_params_->eye_point());
  camera_.lookAt(view_params_->eye_point(), view_params_->look_at());
}


void CiPointCloudViewerApp::updateVerticesVbo() {
  vertices_size_ = 0;
  clouds_->lock();
  for (auto pair : clouds_->clouds()) {
    if (pair.second->vertices().empty() || !pair.second->is_visible()) { continue; }
    if (vertices_vaos_.find(pair.first) == vertices_vaos_.end()) {
      vertices_vaos_[pair.first] = gl::Vao::create();
    }
    if (vertices_vbos_.find(pair.first) == vertices_vbos_.end()) {
      vertices_vbos_[pair.first] = gl::Vbo::create(GL_ARRAY_BUFFER, 0, nullptr, GL_STATIC_DRAW);
    }
    auto vertices = pair.second->vertices();
    vertices_size_ += vertices.size();
    vertices_vbos_[pair.first]->copyData(vertices.size() * sizeof(Vertex), vertices.data());
    gl::ScopedVao vao(vertices_vaos_[pair.first]);
    gl::ScopedBuffer vbo(vertices_vbos_[pair.first]);
    gl::enableVertexAttribArray(0);
    gl::enableVertexAttribArray(1);
    gl::vertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const GLvoid*)offsetof(Vertex, xyz));
    gl::vertexAttribPointer(1, 3, GL_UNSIGNED_BYTE, GL_FALSE, sizeof(Vertex), (const GLvoid*)offsetof(Vertex, rgb));
  }
  clouds_->unlock();
}

void CiPointCloudViewerApp::updatePointsVbo() {
  points_size_ = 0;
  clouds_->lock();
  for (auto pair : clouds_->clouds()) {
    if (pair.second->point_cloud()->empty() || !pair.second->is_visible()) { continue; }
    if (points_vaos_.find(pair.first) == points_vaos_.end()) {
      points_vaos_[pair.first] = gl::Vao::create();
    }
    if (points_vbos_.find(pair.first) == points_vbos_.end()) {
      points_vbos_[pair.first] = gl::Vbo::create(GL_ARRAY_BUFFER, 0, nullptr, GL_STATIC_DRAW);
    }
    auto points = pair.second->point_cloud()->points;
    points_size_ += points.size();
    points_vbos_[pair.first]->copyData(points.size() * sizeof(Cloud::PointT), points.data());
    gl::ScopedVao vao(points_vaos_[pair.first]);
    gl::ScopedBuffer vbo(points_vbos_[pair.first]);
    gl::enableVertexAttribArray(0);
    gl::enableVertexAttribArray(1);
    gl::vertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(Cloud::PointT), (const GLvoid*)offsetof(Cloud::PointT, data));
    gl::vertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_FALSE, sizeof(Cloud::PointT), (const GLvoid*)offsetof(Cloud::PointT, rgba));
  }
  clouds_->unlock();
}

void CiPointCloudViewerApp::mouseDown(MouseEvent event) {
  camera_ui_.mouseDown(event);
  dispatcher_->emit<UpdateCameraParamsAction>({camera_.getEyePoint(), camera_.getPivotPoint()});
}

void CiPointCloudViewerApp::mouseDrag(MouseEvent event) {
  camera_ui_.mouseDrag(event);
  dispatcher_->emit<UpdateCameraParamsAction>({camera_.getEyePoint(), camera_.getPivotPoint()});
}

void CiPointCloudViewerApp::mouseWheel(MouseEvent event) {
  camera_ui_.mouseWheel(event);
  dispatcher_->emit<UpdateCameraParamsAction>({camera_.getEyePoint(), camera_.getPivotPoint()});
}

void CiPointCloudViewerApp::update() {
  gui_->update(this);

  if (!cloud_updated_) {
    updateVerticesVbo();
    updatePointsVbo();
    cloud_updated_ = true;
  }

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

  {
    gl::ScopedGlslProg render(vertices_render_prog_);
    for (auto pair : vertices_vaos_) {
      auto calib_params = clouds_->calib_params_map()[pair.first];
      mat4 calib_matrix(1.0f);
      for (long i = 0; i < calib_params.calib_matrix.cols(); i++) {
        for (long j = 0; j < calib_params.calib_matrix.rows(); j++) {
          calib_matrix[i][j] = calib_params.calib_matrix(j, i);
        }
      }
      vertices_render_prog_->uniform("calibMatrix", calib_matrix);
      vertices_render_prog_->uniform("fx", calib_params.fx);
      vertices_render_prog_->uniform("fy", calib_params.fy);
      vertices_render_prog_->uniform("cx", calib_params.cx);
      vertices_render_prog_->uniform("cy", calib_params.cy);
      vertices_render_prog_->uniform("xPassThroughParams.enable", clouds_->x_pass_through_filter_params().enable);
      vertices_render_prog_->uniform("xPassThroughParams.min", clouds_->x_pass_through_filter_params().min);
      vertices_render_prog_->uniform("xPassThroughParams.max", clouds_->x_pass_through_filter_params().max);
      vertices_render_prog_->uniform("yPassThroughParams.enable", clouds_->y_pass_through_filter_params().enable);
      vertices_render_prog_->uniform("yPassThroughParams.min", clouds_->y_pass_through_filter_params().min);
      vertices_render_prog_->uniform("yPassThroughParams.max", clouds_->y_pass_through_filter_params().max);
      vertices_render_prog_->uniform("zPassThroughParams.enable", clouds_->z_pass_through_filter_params().enable);
      vertices_render_prog_->uniform("zPassThroughParams.min", clouds_->z_pass_through_filter_params().min);
      vertices_render_prog_->uniform("zPassThroughParams.max", clouds_->z_pass_through_filter_params().max);
      gl::ScopedVao vao(pair.second);
      gl::context()->setDefaultShaderVars();
      gl::drawArrays(GL_POINTS, 0, vertices_size_);
    }
  }
  {
    gl::ScopedGlslProg render(points_render_prog_);
    for (auto pair : points_vaos_) {
      gl::ScopedVao vao(pair.second);
      points_render_prog_->uniform("xPassThroughParams.enable", clouds_->x_pass_through_filter_params().enable);
      points_render_prog_->uniform("xPassThroughParams.min", clouds_->x_pass_through_filter_params().min);
      points_render_prog_->uniform("xPassThroughParams.max", clouds_->x_pass_through_filter_params().max);
      points_render_prog_->uniform("yPassThroughParams.enable", clouds_->y_pass_through_filter_params().enable);
      points_render_prog_->uniform("yPassThroughParams.min", clouds_->y_pass_through_filter_params().min);
      points_render_prog_->uniform("yPassThroughParams.max", clouds_->y_pass_through_filter_params().max);
      points_render_prog_->uniform("zPassThroughParams.enable", clouds_->z_pass_through_filter_params().enable);
      points_render_prog_->uniform("zPassThroughParams.min", clouds_->z_pass_through_filter_params().min);
      points_render_prog_->uniform("zPassThroughParams.max", clouds_->z_pass_through_filter_params().max);
      gl::context()->setDefaultShaderVars();
      gl::drawArrays(GL_POINTS, 0, points_size_);
    }
  }
}

CINDER_APP( CiPointCloudViewerApp, RendererGl, [](App::Settings *settings) {
  settings->setHighDensityDisplayEnabled();
  settings->setWindowSize(1280, 960);
})
