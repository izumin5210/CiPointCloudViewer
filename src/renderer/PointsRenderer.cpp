//
// Created by Masayuki IZUMI on 10/5/16.
//

#include "renderer/PointsRenderer.h"

namespace renderer {

PointsRenderer::PointsRenderer(
  ci::app::AppBase *app,
  const std::shared_ptr<Clouds> &clouds
)
  : clouds_(clouds)
  , render_prog_(
    cinder::gl::GlslProg::create(
      cinder::gl::GlslProg::Format()
        .vertex(app->loadAsset("point_cloud.vert"))
        .fragment(app->loadAsset("point_cloud.frag"))
    )
  )
  , size_(0)
{}

void PointsRenderer::update() {
  size_ = 0;
  clouds_->lock();
  for (auto pair : clouds_->clouds()) {
    if (pair.second->point_cloud()->empty() || !pair.second->is_visible()) { continue; }
    if (vaos_.find(pair.first) == vaos_.end()) {
      vaos_[pair.first] = cinder::gl::Vao::create();
    }
    if (vbos_.find(pair.first) == vbos_.end()) {
      vbos_[pair.first] = cinder::gl::Vbo::create(GL_ARRAY_BUFFER, 0, nullptr, GL_STATIC_DRAW);
    }
    auto points = pair.second->point_cloud()->points;
    size_ += points.size();
    vbos_[pair.first]->copyData(points.size() * sizeof(Cloud::PointT), points.data());
    cinder::gl::ScopedVao vao(vaos_[pair.first]);
    cinder::gl::ScopedBuffer vbo(vbos_[pair.first]);
    cinder::gl::enableVertexAttribArray(0);
    cinder::gl::enableVertexAttribArray(1);
    cinder::gl::vertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(Cloud::PointT), (const GLvoid*)offsetof(Cloud::PointT, data));
    cinder::gl::vertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_FALSE, sizeof(Cloud::PointT), (const GLvoid*)offsetof(Cloud::PointT, rgba));
  }
  clouds_->unlock();
}

void PointsRenderer::render() {
  cinder::gl::ScopedGlslProg render(render_prog_);
  for (auto pair : vaos_) {
    cinder::gl::ScopedVao vao(pair.second);
    render_prog_->uniform("xPassThroughParams.enable", clouds_->x_pass_through_filter_params().enable);
    render_prog_->uniform("xPassThroughParams.min", clouds_->x_pass_through_filter_params().min);
    render_prog_->uniform("xPassThroughParams.max", clouds_->x_pass_through_filter_params().max);
    render_prog_->uniform("yPassThroughParams.enable", clouds_->y_pass_through_filter_params().enable);
    render_prog_->uniform("yPassThroughParams.min", clouds_->y_pass_through_filter_params().min);
    render_prog_->uniform("yPassThroughParams.max", clouds_->y_pass_through_filter_params().max);
    render_prog_->uniform("zPassThroughParams.enable", clouds_->z_pass_through_filter_params().enable);
    render_prog_->uniform("zPassThroughParams.min", clouds_->z_pass_through_filter_params().min);
    render_prog_->uniform("zPassThroughParams.max", clouds_->z_pass_through_filter_params().max);
    cinder::gl::context()->setDefaultShaderVars();
    cinder::gl::drawArrays(GL_POINTS, 0, size_);
  }
}

}
