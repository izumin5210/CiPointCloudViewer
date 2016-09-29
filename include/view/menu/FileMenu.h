//
// Created by Masayuki IZUMI on 9/29/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_FILEMENU_H
#define CIPOINTCLOUDVIEWERAPP_FILEMENU_H

#include <boost/filesystem.hpp>

#include "cinder/app/AppBase.h"

#include "Configure.h"
#include "Menu.h"

namespace view {
namespace menu {

class FileMenu : public Menu {
public:
  FileMenu(
    const std::string name,
    ci::app::AppBase *app,
    const std::shared_ptr<Configure> &config
  );


protected:
  void drawImpl() override;


private:
  using path = boost::filesystem::path;

  ci::app::AppBase *app_;
  const std::shared_ptr<Configure> config_;

  void drawOpenPcdFile();
  void drawOpenDirectory();
  void drawOpenCalibYaml();
  void drawOniDirectoryConfig();
  void drawPcdDirectoryConfig();
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_FILEMENU_H
