//
// Created by Masayuki IZUMI on 10/12/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_CLOUDRENDERER_H
#define CIPOINTCLOUDVIEWERAPP_CLOUDRENDERER_H

#include "cinder/app/AppBase.h"

#include "Clouds.h"
#include "renderer/Renderer.h"

namespace renderer {
namespace cloud {

class CloudRenderer : public Renderer {
public:
  CloudRenderer(
    ci::app::AppBase *app,
    const std::shared_ptr<Clouds> &clouds,
    const std::string vertex_shader_name,
    const std::string fragment_shader_name
  );

  void update() override;
  void render() override;


protected:
  virtual void updateVaoAndVbo(const Cloud::Key &key, const std::shared_ptr<Cloud> &cloud) = 0;
  virtual void updateRenderProg(const Cloud::Key &key) = 0;

  std::shared_ptr<Clouds> clouds() const {
    return clouds_;
  }

  cinder::gl::VaoRef& vao(const Cloud::Key key) {
    return vaos_[key].first;
  }

  cinder::gl::VboRef& vbo(const Cloud::Key key) {
    return vaos_[key].second;
  }

  cinder::gl::GlslProgRef render_prog() {
    return render_prog_;
  }

  void add_size(size_t size) {
    size_ += size;
  }


private:
  const std::shared_ptr<Clouds> clouds_;
  const cinder::gl::GlslProgRef render_prog_;
  std::map<Cloud::Key, std::pair<cinder::gl::VaoRef, cinder::gl::VboRef>> vaos_;
  int size_;
  std::set<Cloud::Key> target_keys_;
};

}
}
#endif //CIPOINTCLOUDVIEWERAPP_CLOUDRENDERER_H
