//
// Created by Masayuki IZUMI on 10/6/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_RECTANGULARGRIDRENDERER_H
#define CIPOINTCLOUDVIEWERAPP_RECTANGULARGRIDRENDERER_H

#include "BatchRenderer.h"

namespace renderer {

class RectangularGridRenderer : public BatchRenderer {
public:
  RectangularGridRenderer();


protected:
  cinder::gl::VertBatchRef createBatch() override;
};

}

#endif //CIPOINTCLOUDVIEWERAPP_RECTANGULARGRIDRENDERER_H
