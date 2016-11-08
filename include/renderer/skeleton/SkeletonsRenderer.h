//
// Created by Masayuki IZUMI on 11/3/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_SKELETONSRENDERER_H
#define CIPOINTCLOUDVIEWERAPP_SKELETONSRENDERER_H

#include "cinder/Color.h"

#include "Clouds.h"
#include "renderer/BatchRenderer.h"

namespace renderer {
namespace skeleton {

class SkeletonsRenderer : public BatchRenderer {
public:
  SkeletonsRenderer(const std::shared_ptr<Clouds> &clouds);
  void update() override;


protected:
  cinder::gl::VertBatchRef createBatch() override;


private:
  static const std::map<Bone::Type, cinder::ColorA8u> kBoneColors;

  const std::shared_ptr<Clouds> clouds_;
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_SKELETONSRENDERER_H
