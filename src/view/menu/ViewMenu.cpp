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
  drawGridMenu();
  ui::Separator();
  drawToggleFullScreen();
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
