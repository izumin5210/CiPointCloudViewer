//
// Created by Masayuki IZUMI on 9/29/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_VIEWMENU_H
#define CIPOINTCLOUDVIEWERAPP_VIEWMENU_H

#include "Configure.h"
#include "Menu.h"
#include "ViewParams.h"

namespace view {
namespace menu {

class ViewMenu : public Menu {
public:
  ViewMenu(
    const std::string name,
    const std::shared_ptr<Configure> &config,
    const std::shared_ptr<ViewParams> &view_params
  );


protected:
  void drawImpl() override;


private:
  const std::shared_ptr<Configure> config_;
  const std::shared_ptr<ViewParams> view_params_;

  void drawGridMenu();
  void drawToggleFullScreen();
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_VIEWMENU_H
