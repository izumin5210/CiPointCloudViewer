//
// Created by Masayuki IZUMI on 10/6/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_GRIDRENDERER_H
#define CIPOINTCLOUDVIEWERAPP_GRIDRENDERER_H

#include "renderer/Renderer.h"
#include "ViewParams.h"

namespace renderer {
namespace grid {

class GridRenderer : public Renderer {
public:
  GridRenderer(const std::shared_ptr<ViewParams> &view_params);

  void update() override;

  void render() override;

private:
  const std::shared_ptr<ViewParams> view_params_;
  const std::map<ViewParams::Grid, std::shared_ptr<Renderer>> renderers_;
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_GRIDRENDERER_H
