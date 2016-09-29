//
// Created by Masayuki IZUMI on 9/29/16.
//

#include "view/window/Window.h"

namespace view {
namespace window {

Window::Window(
  const std::string name,
  const int width,
  const int spacing,
  const ImGuiWindowFlags flags
)
  : name_   (name)
  , flags_  (flags)
  , width_  (width)
  , spacing_(spacing)
  , visible_(true)
{}

void Window::draw(glm::vec2 &pos) {
  if (!visible_) { return; }

  ui::ScopedWindow window(name_, flags_);

  drawImpl();

  ui::SetWindowPos(pos);
  ui::SetWindowSize(glm::vec2(width_, 0));
  pos.y += ui::GetWindowHeight() + spacing_;
}

void Window::toggleVisibility() {
  visible_ = !visible_;
}

void Window::drawSpacer() {
  ui::Dummy(glm::vec2(spacing_, spacing_));
}

}
}
