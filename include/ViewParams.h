//
// Created by Masayuki IZUMI on 7/19/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_VIEWPARAMS_H
#define CIPOINTCLOUDVIEWERAPP_VIEWPARAMS_H

#include "cinder/Color.h"

#include "glm/glm.hpp"

#include "Dispatcher.h"
#include "Store.h"

class ViewParams : public Store {
public:
  enum struct Grid {
    NONE        = 0,
    RECTANGULAR = 1,
    POLAR       = 2
  };

  virtual glm::vec3 look_at() const = 0;
  virtual glm::vec3 eye_point() const = 0;
  virtual cinder::Color bg_color() const = 0;
  virtual float point_size() const = 0;
  virtual bool is_full_screen() const = 0;
  virtual Grid grid() const = 0;
};

#endif //CIPOINTCLOUDVIEWERAPP_VIEWPARAMS_H
