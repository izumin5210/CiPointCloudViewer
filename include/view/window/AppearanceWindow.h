//
// Created by Masayuki IZUMI on 9/29/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_APPEARANCEWINDOW_H
#define CIPOINTCLOUDVIEWERAPP_APPEARANCEWINDOW_H

#include "ViewParams.h"
#include "Window.h"

namespace view {
namespace window {

class AppearanceWindow : public Window {
public:
  AppearanceWindow(
    const std::string name,
    const int width,
    const int spacing,
    const ImGuiWindowFlags flags,
    const std::shared_ptr<ViewParams> &view_params
  );


protected:
  void drawImpl() override;


private:
  std::shared_ptr<ViewParams> view_params_;
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_APPEARANCEWINDOW_H
