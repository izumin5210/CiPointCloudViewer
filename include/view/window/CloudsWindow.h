//
// Created by Masayuki IZUMI on 9/29/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_CLOUDSWINDOW_H
#define CIPOINTCLOUDVIEWERAPP_CLOUDSWINDOW_H

#include "Clouds.h"
#include "Window.h"

namespace view {
namespace window {

class CloudsWindow : public Window {
public:
  CloudsWindow(
    const std::string name,
    const int width,
    const int spacing,
    const ImGuiWindowFlags flags,
    const std::shared_ptr<Clouds> &clouds
  );


protected:
  void drawImpl() override;


private:
  const std::shared_ptr<Clouds> clouds_;

  Cloud::Key cloud_selected_;
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_CLOUDSWINDOW_H
