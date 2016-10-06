//
// Created by Masayuki IZUMI on 10/6/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_BATCHRENDERER_H
#define CIPOINTCLOUDVIEWERAPP_BATCHRENDERER_H

#include "cinder/gl/gl.h"
#include "Renderer.h"

namespace renderer {

class BatchRenderer : public Renderer {
public:
  void update() override;
  void render() override;


protected:
  cinder::gl::VertBatchRef batch() {
    return batch_;
  }

  void set_batch(cinder::gl::VertBatchRef batch) {
    batch_ = batch;
  }

  virtual cinder::gl::VertBatchRef createBatch() = 0;


private:
  cinder::gl::VertBatchRef batch_;
};
}

#endif //CIPOINTCLOUDVIEWERAPP_BATCHRENDERER_H
