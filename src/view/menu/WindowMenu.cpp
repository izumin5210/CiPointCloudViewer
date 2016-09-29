//
// Created by Masayuki IZUMI on 9/29/16.
//

#include "view/menu/WindowMenu.h"

namespace view {
namespace menu {

WindowMenu::WindowMenu(
  const std::string name,
  const std::vector<WindowGroup> window_groups
)
  : Menu(name)
  , window_groups_(window_groups)
{}

void WindowMenu::drawImpl() {
  for (auto group : window_groups_) {
    for (auto window : group) {
      drawToggleWindowVisibility(window);
    }
    ui::Separator();
  }
}

void WindowMenu::drawToggleWindowVisibility(std::shared_ptr<Window> &window) {
  if (ui::MenuItem(window->name().c_str(), nullptr, window->visible())) {
    window->toggleVisibility();
  }
}

}
}
