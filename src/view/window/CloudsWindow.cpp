//
// Created by Masayuki IZUMI on 9/29/16.
//

#include "Signal.h"
#include "view/window/CloudsWindow.h"

namespace view {
namespace window {

CloudsWindow::CloudsWindow(
  const std::string name,
  const int width,
  const int spacing,
  const ImGuiWindowFlags flags,
  const std::shared_ptr<Clouds> &clouds
)
  : Window(name, width, spacing, flags)
  , clouds_(clouds)
  , cloud_selected_(std::string())
{}

void CloudsWindow::drawImpl() {
  if (ui::Button("Clear")) {
    Signal<Clouds::ClearCloudsAction>::emit({});
    cloud_selected_ = std::string();
  }

  if (!cloud_selected_.empty()) {
    ui::SameLine();
    if (ui::Button("Remove")) {
      Signal<Clouds::RemoveCloudAction>::emit({cloud_selected_});
      cloud_selected_ = std::string();
    }
  }

  if (!cloud_selected_.empty()) {
    ui::SameLine();
    bool visible = clouds_->clouds()[cloud_selected_]->is_visible();
    if (ui::Button(visible ? "Hide" : "Show")) {
      Signal<Clouds::ChangeCloudVisibilityAction>::emit({cloud_selected_, !visible});
    }
  }

  ui::ListBoxHeader("");
  clouds_->lock();
  for (auto pair : clouds_->clouds()) {
    if (ui::Selectable(pair.first.c_str(), !cloud_selected_.empty() && (cloud_selected_ == pair.first))) {
      cloud_selected_ = pair.first;
    }
  }
  clouds_->unlock();
  ui::ListBoxFooter();
}

}
}