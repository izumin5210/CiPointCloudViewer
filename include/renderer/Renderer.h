//
// Created by Masayuki IZUMI on 10/5/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_RENDERER_H
#define CIPOINTCLOUDVIEWERAPP_RENDERER_H

#include "cinder/gl/gl.h"

namespace renderer {

class Renderer {
public:
  virtual void update() = 0;
  virtual void render() = 0;
};

}

#endif //CIPOINTCLOUDVIEWERAPP_RENDERER_H
