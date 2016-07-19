//
// Created by Masayuki IZUMI on 7/19/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_VIEWPARAMS_H
#define CIPOINTCLOUDVIEWERAPP_VIEWPARAMS_H

#include "glm/glm.hpp"

#include "Store.h"

class ViewParams : public Store {
public:
  struct UpdateCameraParamsAction {
    glm::vec3 eye_point;
    glm::vec3 look_at;
  };

  struct UpdateBgColorAction {
    ci::Color bg_color;
  };

  struct UpdatePointSizeAction {
    float size;
  };

  struct ToggleFullScreenAction {
    bool enable;
  };

  struct ToggleGridVisibilityAction {
    bool visible;
  };

  ViewParams();

  glm::vec3 look_at() const {
    return look_at_;
  }

  glm::vec3 eye_point() const {
    return eye_point_;
  }

  ci::Color bg_color() const {
    return bg_color_;
  }

  float point_size() const {
    return point_size_;
  }

  bool is_full_screen() const {
    return enable_full_screen_;
  }

  bool is_visible_grid() const {
    return visible_grid_;
  }


private:
  glm::vec3 look_at_;
  glm::vec3 eye_point_;
  ci::Color bg_color_;
  float point_size_;
  bool enable_full_screen_;
  bool visible_grid_;

  void initializeConnections();
  void onCameraParamsUpdate(const UpdateCameraParamsAction &action);
  void onBgColorUpdate(const UpdateBgColorAction &action);
  void onPointSizeUpdate(const UpdatePointSizeAction &action);
  void onFullScreenToggle(const ToggleFullScreenAction &action);
  void onGridVisibilityChange(const ToggleGridVisibilityAction &action);
};

#endif //CIPOINTCLOUDVIEWERAPP_VIEWPARAMS_H
