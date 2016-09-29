//
// Created by Masayuki IZUMI on 9/29/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_WINDOW_H
#define CIPOINTCLOUDVIEWERAPP_WINDOW_H

#include "CinderImGui.h"
#include "glm/glm.hpp"

namespace view {
namespace window {

class Window {
public:
  Window(
    const std::string name,
    const int width,
    const int spacing,
    const ImGuiWindowFlags flags
  );

  std::string name() const {
    return name_;
  }

  bool visible() const {
    return visible_;
  }

  void draw(glm::vec2 &pos);
  void toggleVisibility();


protected:
  virtual void drawImpl() = 0;

  void drawSpacer();


private:
  const std::string name_;
  const ImGuiWindowFlags flags_;
  int width_;
  int spacing_;
  bool visible_;
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_WINDOW_H
