//
// Created by Masayuki IZUMI on 9/29/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_PLAYERWINDOW_H
#define CIPOINTCLOUDVIEWERAPP_PLAYERWINDOW_H

#include "Window.h"
#include "io/CapturedLogManager.h"

namespace view {
namespace window {

class PlayerWindow : public Window {
public:
  PlayerWindow(
    const std::string name,
    const int width,
    const int spacing,
    const ImGuiWindowFlags flags,
    const std::shared_ptr<io::CapturedLogManager> &captured_log_manager
  );


protected:
  void drawImpl() override;


private:
  const std::shared_ptr<io::CapturedLogManager> captured_log_manager_;
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_PLAYERWINDOW_H
