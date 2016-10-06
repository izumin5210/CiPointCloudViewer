//
// Created by Masayuki IZUMI on 10/6/16.
//

#include "renderer/grid/GridRenderer.h"
#include "renderer/grid/PolarGridRenderer.h"
#include "renderer/grid/RectangularGridRenderer.h"

namespace renderer {
namespace grid {

GridRenderer::GridRenderer(const std::shared_ptr<ViewParams> &view_params)
  : view_params_(view_params)
  , renderers_({
    { ViewParams::Grid::POLAR, std::make_shared<PolarGridRenderer>() },
    { ViewParams::Grid::RECTANGULAR, std::make_shared<RectangularGridRenderer>() }
  })
{}

void GridRenderer::update() {
  // do nothing.
}

void GridRenderer::render() {
  auto renderer = renderers_.find(view_params_->grid());
  if (renderer != renderers_.end()) {
    renderer->second->render();
  }
}

}
}
