//
// Created by Masayuki IZUMI on 9/29/16.
//

#include "Signal.h"
#include "view/menu/WindowMenu.h"

namespace view {
namespace menu {

WindowMenu::WindowMenu(
  const std::string name,
  const std::shared_ptr<ViewParams> &view_params,
  const std::vector<WindowGroup> window_groups
)
  : Menu(name)
  , view_params_(view_params)
  , window_groups_(window_groups)
{}

void WindowMenu::drawImpl() {
  for (auto group : window_groups_) {
    for (auto window : group) {
      drawToggleWindowVisibility(window);
    }
    ui::Separator();
  }
  auto visible_windows = view_params_->is_windows_visible();
  std::string msg = visible_windows ? "Hide all windows" : "Show all windows";
  if (ui::MenuItem(msg.c_str(), nullptr)) {
    Signal<ViewParams::ToggleWindowsVisibilityAction>::emit({!visible_windows});
  }
}

void WindowMenu::drawToggleWindowVisibility(std::shared_ptr<Window> &window) {
  if (ui::MenuItem(window->name().c_str(), nullptr, window->visible())) {
    window->toggleVisibility();
  }
}

}
}
