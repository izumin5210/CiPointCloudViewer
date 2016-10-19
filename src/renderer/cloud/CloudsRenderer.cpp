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
{
}

void CloudsRenderer::update() {
  points_renderer_->update();
  vertices_renderer_->update();
}

void CloudsRenderer::render() {
  points_renderer_->render();
  vertices_renderer_->render();
}

}
}
