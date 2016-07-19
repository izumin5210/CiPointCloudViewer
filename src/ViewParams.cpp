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
  addConnection(Signal<UpdateCameraParamsAction>::connect(this, &ViewParams::onCameraParamsUpdate));
  addConnection(Signal<UpdateBgColorAction>::connect(this, &ViewParams::onBgColorUpdate));
  addConnection(Signal<UpdatePointSizeAction>::connect(this, &ViewParams::onPointSizeUpdate));
  addConnection(Signal<ToggleFullScreenAction>::connect(this, &ViewParams::onFullScreenToggle));
  addConnection(Signal<ToggleGridVisibilityAction>::connect(this, &ViewParams::onGridVisibilityChange));
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

