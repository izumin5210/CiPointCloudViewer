//
// Created by Masayuki IZUMI on 7/19/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_VIEWPARAMS_H
#define CIPOINTCLOUDVIEWERAPP_VIEWPARAMS_H

#include "cinder/Color.h"
#include "cinder/Camera.h"

#include "glm/glm.hpp"

class ViewParams {
public:
  enum struct Grid {
    NONE        = 0,
    RECTANGULAR = 1,
    POLAR       = 2
  };

  struct UpdateCameraParamsAction {
    glm::vec3 eye_point;
    glm::vec3 look_at;
  };

  struct UpdateBgColorAction {
    cinder::Color bg_color;
  };

  struct UpdatePointSizeAction {
    float size;
  };

  struct ToggleFullScreenAction {
    bool enable;
  };

  struct ToggleWindowsVisibilityAction {
    bool visible;
  };

  struct ChangeGridAction {
    Grid grid;
  };

  ViewParams();

  cinder::CameraPersp& camera() {
    return camera_;
  }

  glm::vec3 look_at() const {
    return camera_.getPivotPoint();
  }

  glm::vec3 eye_point() const {
    return camera_.getEyePoint();
  }

  cinder::Color bg_color() const {
    return bg_color_;
  }

  float point_size() const {
    return point_size_;
  }

  bool is_full_screen() const {
    return enable_full_screen_;
  }

  bool is_windows_visible() const {
    return visible_windows_;
  }

  Grid grid() const {
    return grid_;
  }


private:
  cinder::CameraPersp camera_;
  ci::Color bg_color_;
  float point_size_;
  bool enable_full_screen_;
  bool visible_windows_;
  Grid grid_;

  void initializeConnections();
  void onCameraParamsUpdate(const UpdateCameraParamsAction &action);
  void onBgColorUpdate(const UpdateBgColorAction &action);
  void onPointSizeUpdate(const UpdatePointSizeAction &action);
  void onFullScreenToggle(const ToggleFullScreenAction &action);
  void onWindowVisibilityToggle(const ToggleWindowsVisibilityAction &action);
  void onGridChange(const ChangeGridAction &action);
};

#endif //CIPOINTCLOUDVIEWERAPP_VIEWPARAMS_H
