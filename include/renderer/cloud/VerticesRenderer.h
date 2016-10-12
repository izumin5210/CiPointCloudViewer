//
// Created by Masayuki IZUMI on 10/5/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_VERTICESRENDERER_H
#define CIPOINTCLOUDVIEWERAPP_VERTICESRENDERER_H

#include "CloudRenderer.h"

namespace renderer {
namespace cloud {

class VerticesRenderer : public CloudRenderer {
public:
  VerticesRenderer(
    ci::app::AppBase *app,
    const std::shared_ptr<Clouds> &clouds
  );


protected:
  void updateVaoAndVbo(const Cloud::Key &key, const std::shared_ptr<Cloud> &cloud) override;
  void updateRenderProg(const Cloud::Key &key) override;
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_VERTICESRENDERER_H
