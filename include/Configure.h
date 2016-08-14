//
// Created by Masayuki IZUMI on 7/18/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_CONFIGURE_H
#define CIPOINTCLOUDVIEWERAPP_CONFIGURE_H

#include <boost/filesystem.hpp>

#include <yaml-cpp/yaml.h>

class Configure {
public:
  using bpath = boost::filesystem::path;

  Configure(bpath dir);

  void initialize();

  void setSaveOniFilesTo(std::string dir);
  std::string getSaveOniFilesTo() const;

  void setSavePcdFilesTo(std::string dir);
  std::string getSavePcdFilesTo() const;


private:
  const std::string kFileName = "config.yml";
  const std::string kKeySaveOniFilesTo = "save-oni-files-to";
  const std::string kKeySavePcdFilesTo = "save-pcd-files-to";

  const bpath dir_;
  const bpath path_;

  YAML::Node root_;

  void save();
};

#endif //CIPOINTCLOUDVIEWERAPP_CONFIGURE_H
