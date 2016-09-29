//
// Created by Masayuki IZUMI on 9/29/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_MENU_H
#define CIPOINTCLOUDVIEWERAPP_MENU_H

#include "CinderImGui.h"

namespace view {
namespace menu {

class Menu {
public:
  Menu(const std::string name);

  void draw();


protected:
  virtual void drawImpl() = 0;


private:
  const std::string name_;
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_MENU_H
