//
// Created by Masayuki IZUMI on 7/19/16.
//

#include "ViewParams.h"
#include "Signal.h"

ViewParams::ViewParams()
  : look_at_            (0, 0.5, 0)
  , eye_point_          (4.0, 2.0, -4.0)
  , bg_color_           (cinder::Color8u(0x11, 0x11, 0x11))
  , point_size_         (1.0f)
  , enable_full_screen_ (false)
  , visible_windows_    (true)
  , grid_               (ViewParams::Grid::RECTANGULAR)
{
  initializeConnections();
}

void ViewParams::initializeConnections() {
  namespace ph = std::placeholders;
  Signal<UpdateCameraParamsAction>::connect(std::bind(&ViewParams::onCameraParamsUpdate, this, ph::_1));
  Signal<UpdateBgColorAction>::connect(std::bind(&ViewParams::onBgColorUpdate, this, ph::_1));
  Signal<UpdatePointSizeAction>::connect(std::bind(&ViewParams::onPointSizeUpdate, this, ph::_1));
  Signal<ToggleFullScreenAction>::connect(std::bind(&ViewParams::onFullScreenToggle, this, ph::_1));
  Signal<ToggleWindowsVisibilityAction>::connect(std::bind(&ViewParams::onWindowVisibilityToggle, this, ph::_1));
  Signal<ChangeGridAction>::connect(std::bind(&ViewParams::onGridChange, this, ph::_1));
}


void ViewParams::onCameraParamsUpdate(const UpdateCameraParamsAction &action) {
  eye_point_ = action.eye_point;
  look_at_ = action.look_at;
}

void ViewParams::onBgColorUpdate(const UpdateBgColorAction &action) {
  bg_color_ = action.bg_color;
}

void ViewParams::onPointSizeUpdate(const UpdatePointSizeAction &action) {
  point_size_ = action.size;
}

void ViewParams::onFullScreenToggle(const ToggleFullScreenAction &action) {
  enable_full_screen_ = action.enable;
}

void ViewParams::onWindowVisibilityToggle(const ToggleWindowsVisibilityAction &action) {
  visible_windows_ = action.visible;
}

void ViewParams::onGridChange(const ChangeGridAction &action) {
  grid_ = action.grid;
}
