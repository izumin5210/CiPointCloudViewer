//
// Created by Masayuki IZUMI on 7/18/16.
//

#include <fstream>

#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>

#include "impl/YamlConfigure.h"
#include "action/ViewParamsAction.h"

class YamlConfigure : public Configure {
public:
  using bpath = boost::filesystem::path;

  INJECT(YamlConfigure(
      std::shared_ptr<Dispatcher> dispatcher,
      const cinder::app::AppBase* app
  ))
    : dispatcher_(dispatcher)
    , dir_(app->getAssetPath(""))
    , path_(dir_ / kFileName)
  {
    initialize();
  }

  void initialize() override {
    if (!boost::filesystem::exists(path_)) {
      root_ = YAML::Load("");
    } else {
      root_ = YAML::LoadFile(path_.string());
    }
    dispatcher_->emit<ChangeGridAction>({getGridType()});
  }

  void setSaveOniFilesTo(std::string dir) override {
    root_[kKeySaveOniFilesTo] = dir;
    save();
  }

  std::string getSaveOniFilesTo() const override {
    return root_[kKeySaveOniFilesTo] ? root_[kKeySaveOniFilesTo].as<std::string>() : dir_.string();
  }

  void setSavePcdFilesTo(std::string dir) override {
    root_[kKeySavePcdFilesTo] = dir;
    save();
  }

  std::string getSavePcdFilesTo() const override {
    return root_[kKeySavePcdFilesTo] ? root_[kKeySavePcdFilesTo].as<std::string>() : dir_.string();
  }

  void setSaveGridType(ViewParams::Grid grid) override {
    root_[kKeyGridType] = static_cast<int>(grid);
    save();
    dispatcher_->emit<ChangeGridAction>({grid});
  }

  ViewParams::Grid getGridType() const override {
    return root_[kKeyGridType] ? static_cast<ViewParams::Grid>(root_[kKeyGridType].as<int>()) : ViewParams::Grid::RECTANGULAR;
  }


private:
  const std::string kFileName = "config.yml";
  const std::string kKeySaveOniFilesTo = "save-oni-files-to";
  const std::string kKeySavePcdFilesTo = "save-pcd-files-to";
  const std::string kKeyGridType = "grid";

  std::shared_ptr<Dispatcher> dispatcher_;

  const bpath dir_;
  const bpath path_;

  YAML::Node root_;

  void save() {
    std::ofstream fout(path_.string());
    fout << root_;
    fout.close();
  }
};


fruit::Component<
    fruit::Required<Dispatcher, cinder::app::AppBase>,
    Configure
>
getYamlConfigureComponent() {
  return fruit::createComponent().bind<Configure, YamlConfigure>();
}

