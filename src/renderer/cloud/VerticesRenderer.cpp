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
  : clouds_(clouds)
  , render_prog_(
    cinder::gl::GlslProg::create(
      cinder::gl::GlslProg::Format()
        .vertex(app->loadAsset("vertices.vert"))
        .fragment(app->loadAsset("point_cloud.frag"))
    )
  )
  , size_(0)
{}

void VerticesRenderer::update() {
  size_ = 0;
  target_keys_.clear();
  clouds_->lock();
  for (auto pair : clouds_->clouds()) {
    if (!pair.second->needs_render()) { continue; }
    target_keys_.insert(pair.first);
    if (vaos_.find(pair.first) == vaos_.end()) {
      vaos_[pair.first] = cinder::gl::Vao::create();
    }
    if (vbos_.find(pair.first) == vbos_.end()) {
      vbos_[pair.first] = cinder::gl::Vbo::create(GL_ARRAY_BUFFER, 0, nullptr, GL_STATIC_DRAW);
    }
    auto vertices = pair.second->vertices();
    size_ += vertices->size();
    vbos_[pair.first]->copyData(vertices->size() * sizeof(Vertex), vertices->data());
    cinder::gl::ScopedVao vao(vaos_[pair.first]);
    cinder::gl::ScopedBuffer vbo(vbos_[pair.first]);
    cinder::gl::enableVertexAttribArray(0);
    cinder::gl::enableVertexAttribArray(1);
    cinder::gl::vertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const GLvoid*)offsetof(Vertex, xyz));
    cinder::gl::vertexAttribPointer(1, 3, GL_UNSIGNED_BYTE, GL_FALSE, sizeof(Vertex), (const GLvoid*)offsetof(Vertex, rgb));
  }
  clouds_->unlock();
}

void VerticesRenderer::render() {
  cinder::gl::ScopedGlslProg render(render_prog_);
  for (auto pair : vaos_) {
    if (target_keys_.count(pair.first) == 0) { continue; }
    auto calib_params = clouds_->calib_params_map()[pair.first];
    glm::mat4 calib_matrix(1.0f);
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
    render_prog_->uniform("xPassThroughParams.enable", clouds_->x_pass_through_filter_params().enable);
    render_prog_->uniform("xPassThroughParams.min", clouds_->x_pass_through_filter_params().min);
    render_prog_->uniform("xPassThroughParams.max", clouds_->x_pass_through_filter_params().max);
    render_prog_->uniform("yPassThroughParams.enable", clouds_->y_pass_through_filter_params().enable);
    render_prog_->uniform("yPassThroughParams.min", clouds_->y_pass_through_filter_params().min);
    render_prog_->uniform("yPassThroughParams.max", clouds_->y_pass_through_filter_params().max);
    render_prog_->uniform("zPassThroughParams.enable", clouds_->z_pass_through_filter_params().enable);
    render_prog_->uniform("zPassThroughParams.min", clouds_->z_pass_through_filter_params().min);
    render_prog_->uniform("zPassThroughParams.max", clouds_->z_pass_through_filter_params().max);
    cinder::gl::ScopedVao vao(pair.second);
    cinder::gl::context()->setDefaultShaderVars();
    cinder::gl::drawArrays(GL_POINTS, 0, size_);
  }
}

}
}
