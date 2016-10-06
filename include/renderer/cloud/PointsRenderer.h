//
// Created by Masayuki IZUMI on 10/5/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_POINTSRENDERER_H
#define CIPOINTCLOUDVIEWERAPP_POINTSRENDERER_H

#include "cinder/app/AppBase.h"

#include "Clouds.h"
#include "renderer/Renderer.h"

namespace renderer {
namespace cloud {

class PointsRenderer : public Renderer {
public:
  PointsRenderer(
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

#endif //CIPOINTCLOUDVIEWERAPP_POINTSRENDERER_H
