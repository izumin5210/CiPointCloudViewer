//
// Created by Masayuki IZUMI on 10/5/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_CLOUDRENDERER_H
#define CIPOINTCLOUDVIEWERAPP_CLOUDRENDERER_H

#include "cinder/app/AppBase.h"

#include "Clouds.h"
#include "Renderer.h"

namespace renderer {

class CloudsRenderer : public Renderer {
public:
  CloudsRenderer(
    ci::app::AppBase *app,
    const std::shared_ptr<Clouds> &clouds
  );

  void update() override;
  void render() override;


private:
  const std::unique_ptr<Renderer> points_renderer_;
  const std::unique_ptr<Renderer> vertices_renderer_;
  const std::shared_ptr<Clouds> clouds_;

  std::atomic<bool> needs_to_update_;

  void initialize();
  void onCloudsUpdate();
};

}

#endif //CIPOINTCLOUDVIEWERAPP_CLOUDRENDERER_H
