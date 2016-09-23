//
// Created by Masayuki IZUMI on 7/18/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_APPGUI_H
#define CIPOINTCLOUDVIEWERAPP_APPGUI_H

#include "cinder/app/AppBase.h"

class AppGui {
public:
  virtual void initialize() = 0;
  virtual void update(ci::app::AppBase *app) = 0;
};

#endif //CIPOINTCLOUDVIEWERAPP_APPGUI_H
