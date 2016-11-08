//
// Created by Masayuki IZUMI on 11/3/16.
//

#include "glm/glm.hpp"
#include "renderer/skeleton/SkeletonsRenderer.h"

namespace renderer {
namespace skeleton {

SkeletonsRenderer::SkeletonsRenderer(
  const std::shared_ptr<Clouds> &clouds
)
  : BatchRenderer()
  , clouds_(clouds)
{
  set_batch(createBatch());
}

void SkeletonsRenderer::update() {
  batch()->clear();
  for (auto p1 : clouds_->clouds()) {
    if (!p1.second->hasSkeleton()) { continue; }
    auto skeleton = p1.second->skeleton();
    for (auto p2 : Bone::joint_pairs) {
      if (skeleton.count(p2.second.first) != 0 && skeleton.count(p2.second.second) != 0) {
        auto j1 = skeleton[p2.second.first];
        auto j2 = skeleton[p2.second.second];
        batch()->color(kBoneColors.at(p2.first));
        batch()->vertex(glm::vec3(j1.x, j1.y, j1.z));
        batch()->vertex(glm::vec3(j2.x, j2.y, j2.z));
      }
    }
  }
}

cinder::gl::VertBatchRef SkeletonsRenderer::createBatch() {
  return cinder::gl::VertBatch::create(GL_LINES);
}

const std::map<Bone::Type, cinder::ColorA8u> SkeletonsRenderer::kBoneColors = {
  { Bone::Type::HEAD,            cinder::ColorA8u(0xcc, 0x66, 0x66, 0xcc) },
  { Bone::Type::TORSO,           cinder::ColorA8u(0x33, 0x33, 0xcc, 0xcc) },
  { Bone::Type::LEFT_UP_ARM,     cinder::ColorA8u(0xcc, 0xcc, 0x99, 0xcc) },
  { Bone::Type::RIGHT_UP_ARM,    cinder::ColorA8u(0xff, 0x66, 0xcc, 0xcc) },
  { Bone::Type::LEFT_DOWN_ARM,   cinder::ColorA8u(0x33, 0x99, 0xff, 0xcc) },
  { Bone::Type::RIGHT_DOWN_ARM,  cinder::ColorA8u(0x66, 0xcc, 0x66, 0xcc) },
  { Bone::Type::LEFT_UP_LEG,     cinder::ColorA8u(0xcc, 0xcc, 0x99, 0xcc) },
  { Bone::Type::RIGHT_UP_LEG,    cinder::ColorA8u(0xff, 0x66, 0xcc, 0xcc) },
  { Bone::Type::LEFT_DOWN_LEG,   cinder::ColorA8u(0x33, 0x99, 0xff, 0xcc) },
  { Bone::Type::RIGHT_DOWN_LEG,  cinder::ColorA8u(0x66, 0xcc, 0x66, 0xcc) }
};

}
}
