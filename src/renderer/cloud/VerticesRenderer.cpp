//
// Created by Masayuki IZUMI on 10/5/16.
//

#include "glm/glm.hpp"
#include "renderer/cloud/VerticesRenderer.h"

namespace renderer {
namespace cloud {

VerticesRenderer::VerticesRenderer(
  ci::app::AppBase *app,
  const std::shared_ptr<Clouds> &clouds
)
  : CloudRenderer(app, clouds, "vertices.vert", "point_cloud.frag")
{}

void VerticesRenderer::updateVaoAndVbo(const Cloud::Key &key, const std::shared_ptr<Cloud> &cloud) {
  auto vertices = cloud->vertices();
  set_size(key, vertices->size());
  vbo(key)->copyData(vertices->size() * sizeof(Vertex), vertices->data());
  cinder::gl::ScopedVao svao(vao(key));
  cinder::gl::ScopedBuffer svbo(vbo(key));
  cinder::gl::enableVertexAttribArray(0);
  cinder::gl::enableVertexAttribArray(1);
  cinder::gl::vertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const GLvoid*)offsetof(Vertex, xyz));
  cinder::gl::vertexAttribPointer(1, 3, GL_UNSIGNED_BYTE, GL_FALSE, sizeof(Vertex), (const GLvoid*)offsetof(Vertex, rgb));
}

void VerticesRenderer::updateRenderProg(const Cloud::Key &key) {
  auto calib_params = clouds()->calib_params_map()[key];
  glm::mat4 calib_matrix(1.0f);
  for (long i = 0; i < calib_params.calib_matrix.cols(); i++) {
    for (long j = 0; j < calib_params.calib_matrix.rows(); j++) {
      calib_matrix[i][j] = calib_params.calib_matrix(j, i);
    }
  }
  render_prog()->uniform("calibMatrix", calib_matrix);
  render_prog()->uniform("fx", calib_params.fx);
  render_prog()->uniform("fy", calib_params.fy);
  render_prog()->uniform("cx", calib_params.cx);
  render_prog()->uniform("cy", calib_params.cy);
}

}
}
