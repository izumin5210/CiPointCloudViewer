//
// Created by Masayuki IZUMI on 7/18/16.
//

#include "Configure.h"

Configure::Configure(bpath dir)
  : dir_(dir)
  , path_(dir_ / kFileName)
{
}

void Configure::initialize() {
  if (!boost::filesystem::exists(path_)) {
    boost::filesystem::save_string_file(path_.string(), "");
  } else {
    root_ = YAML::LoadFile(path_.string());
  }
}

void Configure::setSaveOniFilesTo(std::string dir) {
  root_[kKeySaveOniFilesTo] = dir;
  save();
}

std::string Configure::getSaveOniFilesTo() {
  return root_[kKeySaveOniFilesTo] ? root_[kKeySaveOniFilesTo].as<std::string>() : dir_.string();
}

void Configure::save() {
  std::ofstream fout(path_.string());
  fout << root_;
  fout.close();
}
