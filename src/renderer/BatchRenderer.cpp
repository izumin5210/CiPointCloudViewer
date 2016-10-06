//
// Created by Masayuki IZUMI on 10/6/16.
//

#include "renderer/BatchRenderer.h"

namespace renderer {

void BatchRenderer::update() {
  // do nothing.
}

void BatchRenderer::render() {
  batch_->draw();
}

}
