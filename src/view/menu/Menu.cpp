//
// Created by Masayuki IZUMI on 9/29/16.
//

#include "view/menu/Menu.h"

namespace view {
namespace menu {

Menu::Menu(const std::string name)
  : name_(name)
{}

void Menu::draw() {
  if (ui::BeginMenu(name_.c_str())) {
    drawImpl();
    ui::EndMenu();
  }
}

}
}
