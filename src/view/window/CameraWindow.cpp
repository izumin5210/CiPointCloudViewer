//
// Created by Masayuki IZUMI on 9/29/16.
//

#include "Signal.h"
#include "view/window/CameraWindow.h"

namespace view {
namespace window {

CameraWindow::CameraWindow(
  const std::string name,
  const int width,
  const int spacing,
  const ImGuiWindowFlags flags,
  const std::shared_ptr<ViewParams> &view_params
)
  : Window(name, width, spacing, flags)
  , view_params_(view_params)
{}

void CameraWindow::drawImpl() {
  auto look_at = view_params_->look_at();
  if (ui::DragFloat3("Look at", &look_at[0])) {
    Signal<ViewParams::UpdateCameraParamsAction>::emit({view_params_->eye_point(), look_at});
  }
  auto eye_point = view_params_->eye_point();
  if (ui::DragFloat3("Eye point", &eye_point[0])) {
    Signal<ViewParams::UpdateCameraParamsAction>::emit({eye_point, view_params_->look_at()});
  }
}

}
}
