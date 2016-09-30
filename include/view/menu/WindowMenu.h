//
// Created by Masayuki IZUMI on 9/29/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_WINDOWMENU_H
#define CIPOINTCLOUDVIEWERAPP_WINDOWMENU_H

#include "Menu.h"
#include "ViewParams.h"
#include "view/window/Window.h"

namespace view {
namespace menu {

class WindowMenu : public Menu {
public:
  using Window = view::window::Window;
  using WindowGroup = std::vector<std::shared_ptr<Window>>;

  WindowMenu(
    const std::string name,
    const std::shared_ptr<ViewParams> &view_params,
    const std::vector<WindowGroup> window_groups
  );


protected:
  void drawImpl() override;


private:
  const std::shared_ptr<ViewParams> view_params_;
  const std::vector<WindowGroup> window_groups_;

  void drawToggleWindowVisibility(std::shared_ptr<Window> &window);
};


}
}

#endif //CIPOINTCLOUDVIEWERAPP_WINDOWMENU_H
