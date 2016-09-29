//
// Created by Masayuki IZUMI on 9/29/16.
//

#include "view/window/InformationWindow.h"

namespace view {
namespace window {

InformationWindow::InformationWindow(
  const std::string name,
  const int width,
  const int spacing,
  const ImGuiWindowFlags flags,
  const ci::app::AppBase *app,
  const std::shared_ptr<Clouds> &clouds
)
  : Window(name, width, spacing, flags)
  , app_(app)
  , clouds_(clouds)
{}

void InformationWindow::drawImpl() {
  ui::LabelText("FPS", "%f", app_->getAverageFps());
  ui::LabelText("points", "%zu", clouds_->size());
}

}
}
