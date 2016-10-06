//
// Created by Masayuki IZUMI on 10/6/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_RECTANGULARGRIDRENDERER_H
#define CIPOINTCLOUDVIEWERAPP_RECTANGULARGRIDRENDERER_H

#include "renderer/BatchRenderer.h"

namespace renderer {
namespace grid {

class RectangularGridRenderer : public BatchRenderer {
public:
  RectangularGridRenderer();


protected:
  cinder::gl::VertBatchRef createBatch() override;
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_RECTANGULARGRIDRENDERER_H
