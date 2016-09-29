//
// Created by Masayuki IZUMI on 9/29/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_INFORMATIONWINDOW_H
#define CIPOINTCLOUDVIEWERAPP_INFORMATIONWINDOW_H

#include "cinder/app/AppBase.h"

#include "Clouds.h"
#include "Window.h"

namespace view {
namespace window {

class InformationWindow : public Window {
public:
  InformationWindow(
    const std::string name,
    const int width,
    const int spacing,
    const ImGuiWindowFlags flags,
    const ci::app::AppBase *app,
    const std::shared_ptr<Clouds> &clouds
  );


protected:
  void drawImpl() override;


private:
  const ci::app::AppBase *app_;
  const std::shared_ptr<Clouds> clouds_;
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_INFORMATIONWINDOW_H
