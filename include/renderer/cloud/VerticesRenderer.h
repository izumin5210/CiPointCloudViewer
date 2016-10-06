//
// Created by Masayuki IZUMI on 10/5/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_VERTICESRENDERER_H
#define CIPOINTCLOUDVIEWERAPP_VERTICESRENDERER_H

#include "cinder/app/AppBase.h"

#include "Clouds.h"
#include "renderer/Renderer.h"

namespace renderer {
namespace cloud {


class VerticesRenderer : public Renderer {
public:
  VerticesRenderer(
    ci::app::AppBase *app,
    const std::shared_ptr<Clouds> &clouds
  );

  void update() override;
  void render() override;


private:
  const std::shared_ptr<Clouds> clouds_;
  const cinder::gl::GlslProgRef render_prog_;
  std::map<Cloud::Key, cinder::gl::VaoRef> vaos_;
  std::map<Cloud::Key, cinder::gl::VboRef> vbos_;
  int size_;
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_VERTICESRENDERER_H
