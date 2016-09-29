//
// Created by Masayuki IZUMI on 9/29/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_CAMERAWINDOW_H
#define CIPOINTCLOUDVIEWERAPP_CAMERAWINDOW_H

#include "ViewParams.h"
#include "Window.h"

namespace view {
namespace window {

class CameraWindow : public Window {
public:
  CameraWindow(
    const std::string name,
    const int width,
    const int spacing,
    const ImGuiWindowFlags flags,
    const std::shared_ptr<ViewParams> &view_params
  );


protected:
  void drawImpl() override;


private:
  const std::shared_ptr<ViewParams> view_params_;
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_CAMERAWINDOW_H
