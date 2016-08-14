//
// Created by Masayuki IZUMI on 7/18/16.
//

#include <fstream>
#include "Configure.h"

Configure::Configure(bpath dir)
  : dir_(dir)
  , path_(dir_ / kFileName)
{
}

void Configure::initialize() {
  if (!boost::filesystem::exists(path_)) {
    root_ = YAML::Load("");
  } else {
    root_ = YAML::LoadFile(path_.string());
  }
}

void Configure::setSaveOniFilesTo(std::string dir) {
  root_[kKeySaveOniFilesTo] = dir;
  save();
}

std::string Configure::getSaveOniFilesTo() const {
  return root_[kKeySaveOniFilesTo] ? root_[kKeySaveOniFilesTo].as<std::string>() : dir_.string();
}

void Configure::setSavePcdFilesTo(std::string dir) {
  root_[kKeySavePcdFilesTo] = dir;
  save();
}

std::string Configure::getSavePcdFilesTo() const {
  return root_[kKeySavePcdFilesTo] ? root_[kKeySavePcdFilesTo].as<std::string>() : dir_.string();
}

void Configure::save() {
  std::ofstream fout(path_.string());
  fout << root_;
  fout.close();
}
