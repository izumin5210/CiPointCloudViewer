//
// Created by Masayuki IZUMI on 10/6/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_POLARGRIDRENDERER_H
#define CIPOINTCLOUDVIEWERAPP_POLARGRIDRENDERER_H

#include "renderer/BatchRenderer.h"

namespace renderer {
namespace grid {

class PolarGridRenderer : public BatchRenderer {
public:
  PolarGridRenderer();


protected:
  cinder::gl::VertBatchRef createBatch() override;
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_POLARGRIDRENDERER_H
