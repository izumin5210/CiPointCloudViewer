//
// Created by Masayuki IZUMI on 9/29/16.
//

#include "Clouds.h"
#include "Signal.h"
#include "io/CalibrationParamsManager.h"
#include "io/CloudDataSources.h"
#include "view/menu/FileMenu.h"

namespace view {
namespace menu {

FileMenu::FileMenu(
  const std::string name,
  ci::app::AppBase *app,
  const std::shared_ptr<Configure> &config
)
  : Menu(name)
  , app_(app)
  , config_(config)
{}

void FileMenu::drawImpl() {
  drawOpenPcdFile();
  drawOpenDirectory();

  ui::Separator();

  drawOniDirectoryConfig();
  drawPcdDirectoryConfig();
}

void FileMenu::drawOpenPcdFile() {
  if (ui::MenuItem("Open *.pcd file")) {
    auto pcdfile = app_->getOpenFilePath(path(), {"pcd"});
    if (boost::filesystem::exists(pcdfile)) {
      Signal<Clouds::OpenPcdFileAction>::emit({pcdfile.string()});
    }
  }
}

void FileMenu::drawOpenDirectory() {
  if(ui::MenuItem("Open directory")) {
    auto dir = app_->getFolderPath();
    if (boost::filesystem::is_directory(dir)) {
      Signal<io::CloudDataSources::OpenPcdFilesDirectoryAction>::emit({dir.string()});
    }
  }
}

void FileMenu::drawOpenCalibYaml() {
  if (ui::MenuItem("Open calibration yaml file")) {
    auto yamlfile = app_->getOpenFilePath(path(), {"yaml", "yml"});
    if (boost::filesystem::exists(yamlfile)) {
      io::CalibrationParams::load(yamlfile.string());
    }
  }
}


void FileMenu::drawOniDirectoryConfig() {
  if (ui::MenuItem("Save *.oni to ...")) {
    auto dir = app_->getFolderPath(path(config_->getSaveOniFilesTo()));
    if (boost::filesystem::is_directory(dir)) {
      config_->setSaveOniFilesTo(dir.string());
    }
  }
}

void FileMenu::drawPcdDirectoryConfig() {
  if (ui::MenuItem("Save *.pcd to ...")) {
    auto dir = app_->getFolderPath(path(config_->getSavePcdFilesTo()));
    if (boost::filesystem::is_directory(dir)) {
      config_->setSavePcdFilesTo(dir.string());
    }
  }
}

}
}

