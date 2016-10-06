//
// Created by Masayuki IZUMI on 10/6/16.
//

#include "glm/glm.hpp"
#include "renderer/grid/PolarGridRenderer.h"

namespace renderer {
namespace grid {

PolarGridRenderer::PolarGridRenderer() {
  set_batch(createBatch());
}

cinder::gl::VertBatchRef PolarGridRenderer::createBatch() {
  auto batch = cinder::gl::VertBatch::create(GL_LINE_LOOP);
  for (int i = 0; i < 5; i++) {
    batch->color(1, 1, 1, 0.3);
    for (int j = 0; j < 360; j++) {
      float rad = (float) (j * M_PI) / 180;
      batch->vertex((i + 1) * cos(rad), 0, (i + 1) * sin(rad));
    }
  }
  batch->setType(GL_LINES);
  batch->color(1, 1, 1, 0.3);
  for (int i = 0; i < 6; i++) {
    auto x = 5 * cos(M_PI / 6 * i);
    auto z = 5 * sin(M_PI / 6 * i);
    batch->vertex(glm::vec3(x, 0, z));
    batch->vertex(glm::vec3(-x, 0, -z));
  }
  return batch;
}

}
}
