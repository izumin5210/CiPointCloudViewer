//
// Created by Masayuki IZUMI on 2016/09/02.
//

#ifndef CIPOINTCLOUDVIEWERAPP_VIEWPARAMSACTION_H
#define CIPOINTCLOUDVIEWERAPP_VIEWPARAMSACTION_H

#include "ViewParams.h"

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

struct ChangeGridAction {
  ViewParams::Grid grid;
};

#endif //CIPOINTCLOUDVIEWERAPP_VIEWPARAMSACTION_H
