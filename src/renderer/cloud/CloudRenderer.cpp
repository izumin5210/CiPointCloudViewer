//
// Created by Masayuki IZUMI on 10/12/16.
//

#include "renderer/cloud/CloudRenderer.h"

namespace renderer {
namespace cloud {

CloudRenderer::CloudRenderer(
  ci::app::AppBase *app,
  const std::shared_ptr<Clouds> &clouds,
  const std::string vertex_shader_name,
  const std::string fragment_shader_name
)
  : clouds_(clouds)
  , render_prog_(
    cinder::gl::GlslProg::create(
      cinder::gl::GlslProg::Format()
        .vertex(app->loadAsset(vertex_shader_name))
        .fragment(app->loadAsset(fragment_shader_name))
    )
  )
  , size_(0)
{}

void CloudRenderer::update() {
  size_ = 0;
  target_keys_.clear();
  clouds_->lock();
  for (auto pair : clouds_->clouds()) {
    if (!pair.second->needs_render()) { continue; }
    target_keys_.insert(pair.first);
    if (vaos_.count(pair.first) == 0) {
      auto vao = cinder::gl::Vao::create();
      auto vbo = cinder::gl::Vbo::create(GL_ARRAY_BUFFER, 0, nullptr, GL_STATIC_DRAW);
      vaos_[pair.first] = std::make_pair(vao, vbo);
    }
    updateVaoAndVbo(pair.first, pair.second);
  }
  clearUnusedVaos();
  clouds_->unlock();
}

void CloudRenderer::render() {
  cinder::gl::ScopedGlslProg render(render_prog_);
  render_prog_->uniform("xPassThroughParams.enable", clouds_->x_pass_through_filter_params().enable);
  render_prog_->uniform("xPassThroughParams.min", clouds_->x_pass_through_filter_params().min);
  render_prog_->uniform("xPassThroughParams.max", clouds_->x_pass_through_filter_params().max);
  render_prog_->uniform("yPassThroughParams.enable", clouds_->y_pass_through_filter_params().enable);
  render_prog_->uniform("yPassThroughParams.min", clouds_->y_pass_through_filter_params().min);
  render_prog_->uniform("yPassThroughParams.max", clouds_->y_pass_through_filter_params().max);
  render_prog_->uniform("zPassThroughParams.enable", clouds_->z_pass_through_filter_params().enable);
  render_prog_->uniform("zPassThroughParams.min", clouds_->z_pass_through_filter_params().min);
  render_prog_->uniform("zPassThroughParams.max", clouds_->z_pass_through_filter_params().max);
  for (auto pair : vaos_) {
    if (target_keys_.count(pair.first) == 0) { continue; }
    updateRenderProg(pair.first);
    cinder::gl::ScopedVao svao(pair.second.first);
    cinder::gl::context()->setDefaultShaderVars();
    cinder::gl::drawArrays(GL_POINTS, 0, size_);
  }
}

void CloudRenderer::clearUnusedVaos() {
  std::vector<Cloud::Key> deleted_keys;
  for (auto pair : vaos_) {
    if (clouds_->clouds().count(pair.first) == 0) {
      deleted_keys.emplace_back(pair.first);
    }
  }
  for (auto key : deleted_keys) {
    vaos_.erase(key);
  }
}

}
}
