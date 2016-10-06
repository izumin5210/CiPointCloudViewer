//
// Created by Masayuki IZUMI on 10/6/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_POLARGRIDRENDERER_H
#define CIPOINTCLOUDVIEWERAPP_POLARGRIDRENDERER_H

#include "BatchRenderer.h"

namespace renderer {

class PolarGridRenderer : public BatchRenderer {
public:
  PolarGridRenderer();


protected:
  cinder::gl::VertBatchRef createBatch() override;
};

}

#endif //CIPOINTCLOUDVIEWERAPP_POLARGRIDRENDERER_H
