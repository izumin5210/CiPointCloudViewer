//
// Created by Masayuki IZUMI on 10/6/16.
//

#include "glm/glm.hpp"
#include "renderer/RectangularGridRenderer.h"

namespace renderer {

RectangularGridRenderer::RectangularGridRenderer() {
  set_batch(createBatch());
}

cinder::gl::VertBatchRef RectangularGridRenderer::createBatch() {
  auto batch = cinder::gl::VertBatch::create(GL_LINES);
  batch->color(1, 1, 1, 0.3);
  for (float i = -5; i <= 5.0; i += 0.5) {
    for (float j = -5; j <= 5.0; j += 0.5) {
      batch->vertex(glm::vec3( i, 0, -j));
      batch->vertex(glm::vec3( i, 0,  j));
      batch->vertex(glm::vec3(-i, 0,  j));
      batch->vertex(glm::vec3( i, 0,  j));
    }
  }
  return batch;
}

}
