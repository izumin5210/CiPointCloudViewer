//
// Created by Masayuki IZUMI on 10/5/16.
//

#include "renderer/cloud/PointsRenderer.h"

namespace renderer {
namespace cloud {

PointsRenderer::PointsRenderer(
  ci::app::AppBase *app,
  const std::shared_ptr<Clouds> &clouds
)
  : CloudRenderer(app, clouds, "point_cloud.vert", "point_cloud.frag")
{}

void PointsRenderer::updateVaoAndVbo(const Cloud::Key &key, const std::shared_ptr<Cloud> &cloud) {
  auto size = cloud->point_cloud()->size();
  set_size(key, size);
  vbo(key)->copyData(size * sizeof(Cloud::PointT), cloud->point_cloud()->points.data());
  cinder::gl::ScopedVao svao(vao(key));
  cinder::gl::ScopedBuffer svbo(vbo(key));
  cinder::gl::enableVertexAttribArray(0);
  cinder::gl::enableVertexAttribArray(1);
  cinder::gl::vertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(Cloud::PointT), (const GLvoid*)offsetof(Cloud::PointT, data));
  cinder::gl::vertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_FALSE, sizeof(Cloud::PointT), (const GLvoid*)offsetof(Cloud::PointT, rgba));
}

void PointsRenderer::updateRenderProg(const Cloud::Key &key) {
  auto calibrated = clouds()->clouds()[key]->is_calibrated();
  if (calibrated) { return; }
  auto calib_params = clouds()->calib_params_map()[key];
  glm::mat4 calib_matrix(1.0f);
  for (long i = 0; i < calib_params.calib_matrix.cols(); i++) {
    for (long j = 0; j < calib_params.calib_matrix.rows(); j++) {
      calib_matrix[i][j] = calib_params.calib_matrix(j, i);
    }
  }
  render_prog()->uniform("calibrated", calibrated);
  render_prog()->uniform("calibMatrix", calib_matrix);
  render_prog()->uniform("fx", calib_params.fx);
  render_prog()->uniform("fy", calib_params.fy);
  render_prog()->uniform("cx", calib_params.cx);
  render_prog()->uniform("cy", calib_params.cy);
}

}
}
