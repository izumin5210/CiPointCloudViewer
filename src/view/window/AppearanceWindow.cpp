//
// Created by Masayuki IZUMI on 9/29/16.
//

#include "Signal.h"
#include "view/window/AppearanceWindow.h"

namespace view {
namespace window {

AppearanceWindow::AppearanceWindow(
  const std::string name,
  const int width,
  const int spacing,
  const ImGuiWindowFlags flags,
  const std::shared_ptr<ViewParams> &view_params
)
  : Window(name, width, spacing, flags)
  , view_params_(view_params)
{}

void AppearanceWindow::drawImpl() {
  float point_size = view_params_->point_size();
  if (ui::InputFloat("Point size", &point_size, 0.1f)) {
    Signal<ViewParams::UpdatePointSizeAction>::emit({point_size});
  }
  ci::Color bg_color = view_params_->bg_color();
  if (ui::ColorEdit3("Background", &bg_color[0])) {
    Signal<ViewParams::UpdateBgColorAction>::emit({bg_color});
  }
}

}
}
