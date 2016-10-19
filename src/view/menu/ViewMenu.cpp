//
// Created by Masayuki IZUMI on 9/29/16.
//

#include "Signal.h"
#include "view/menu/ViewMenu.h"

namespace view {
namespace menu {

ViewMenu::ViewMenu(
  const std::string name,
  const std::shared_ptr<Configure> &config,
  const std::shared_ptr<ViewParams> &view_params
)
  : Menu(name)
  , config_(config)
  , view_params_(view_params)
{}

void ViewMenu::drawImpl() {
  drawBackground();
  drawGridMenu();
  ui::Separator();
  drawToggleFullScreen();
}

void ViewMenu::drawBackground() {
  if (ui::BeginMenu("Background color")) {
    std::map<std::string, cinder::Color> map = {
      { "#111111", cinder::Color8u(0x11, 0x11, 0x11) },
      { "#333333", cinder::Color8u(0x33, 0x33, 0x33) },
      { "#666666", cinder::Color8u(0x66, 0x66, 0x66) },
      { "#999999", cinder::Color8u(0x99, 0x99, 0x99) }
    };
    for (auto pair : map) {
      if (ui::MenuItem(pair.first.c_str())) {
        Signal<ViewParams::UpdateBgColorAction>::emit({pair.second});
      }
    }
    ui::EndMenu();
  }
}

void ViewMenu::drawGridMenu() {
  if (ui::BeginMenu("Grid")) {
    if (ui::MenuItem("None", nullptr, view_params_->grid() == ViewParams::Grid::NONE)) {
      config_->setSaveGridType(ViewParams::Grid::NONE);
    }
    if (ui::MenuItem("Rectangular grid", nullptr, view_params_->grid() == ViewParams::Grid::RECTANGULAR)) {
      config_->setSaveGridType(ViewParams::Grid::RECTANGULAR);
    }
    if (ui::MenuItem("Polar grid", nullptr, view_params_->grid() == ViewParams::Grid::POLAR)) {
      config_->setSaveGridType(ViewParams::Grid::POLAR);
    }
    ui::EndMenu();
  }
}

void ViewMenu::drawToggleFullScreen() {
  if (ui::MenuItem("Full screen", nullptr, view_params_->is_full_screen())) {
    Signal<ViewParams::ToggleFullScreenAction>::emit({!view_params_->is_full_screen()});
  }
}

}
}
