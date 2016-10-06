//
// Created by Masayuki IZUMI on 10/5/16.
//

#include "renderer/cloud/CloudsRenderer.h"
#include "renderer/cloud/PointsRenderer.h"
#include "renderer/cloud/VerticesRenderer.h"

namespace renderer {
namespace cloud {

CloudsRenderer::CloudsRenderer(
  ci::app::AppBase *app,
  const std::shared_ptr<Clouds> &clouds
)
  : points_renderer_(new PointsRenderer(app, clouds))
  , vertices_renderer_(new VerticesRenderer(app, clouds))
  , clouds_(clouds)
  , needs_to_update_(true)
{
  initialize();
}

void CloudsRenderer::update() {
  if (needs_to_update_) {
    points_renderer_->update();
    vertices_renderer_->update();
    needs_to_update_ = false;
  }
}

void CloudsRenderer::render() {
  points_renderer_->render();
  vertices_renderer_->render();
}

void CloudsRenderer::initialize() {
  clouds_->connect(std::bind(&CloudsRenderer::onCloudsUpdate, this));
}

void CloudsRenderer::onCloudsUpdate() {
  needs_to_update_ = true;
}

}
}
