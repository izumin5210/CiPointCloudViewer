//
// Created by Masayuki IZUMI on 7/19/16.
//

#include "cinder/Color.h"

#include "ViewParams.h"
#include "Signal.h"

ViewParams::ViewParams()
  : look_at_            (0, 0.5, 0)
  , eye_point_          (4.0, 2.0, -4.0)
  , bg_color_           (cinder::Color8u(0x11, 0x11, 0x11))
  , point_size_         (1.0f)
  , enable_full_screen_ (false)
  , visible_grid_       (true)
{
  initializeConnections();
}

void ViewParams::initializeConnections() {
  namespace ph = std::placeholders;
  addConnection(Signal<UpdateCameraParamsAction>::connect(std::bind(&ViewParams::onCameraParamsUpdate, this, ph::_1)));
  addConnection(Signal<UpdateBgColorAction>::connect(std::bind(&ViewParams::onBgColorUpdate, this, ph::_1)));
  addConnection(Signal<UpdatePointSizeAction>::connect(std::bind(&ViewParams::onPointSizeUpdate, this, ph::_1)));
  addConnection(Signal<ToggleFullScreenAction>::connect(std::bind(&ViewParams::onFullScreenToggle, this, ph::_1)));
  addConnection(Signal<ToggleGridVisibilityAction>::connect(std::bind(&ViewParams::onGridVisibilityChange, this, ph::_1)));
}


void ViewParams::onCameraParamsUpdate(const UpdateCameraParamsAction &action) {
  eye_point_ = action.eye_point;
  look_at_ = action.look_at;
  emit();
}

void ViewParams::onBgColorUpdate(const UpdateBgColorAction &action) {
  bg_color_ = action.bg_color;
  emit();
}

void ViewParams::onPointSizeUpdate(const UpdatePointSizeAction &action) {
  point_size_ = action.size;
  emit();
}

void ViewParams::onFullScreenToggle(const ToggleFullScreenAction &action) {
  enable_full_screen_ = action.enable;
  emit();
}

void ViewParams::onGridVisibilityChange(const ToggleGridVisibilityAction &action) {
  visible_grid_ = action.visible;
  emit();
}

