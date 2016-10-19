//
// Created by Masayuki IZUMI on 7/20/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_CLOUDDATASOURCES_H
#define CIPOINTCLOUDVIEWERAPP_CLOUDDATASOURCES_H

#include <map>

#include "SequentialPcdPlayer.h"

namespace io {

class CloudDataSources {
public:
  using Key = std::string;

  struct OpenPcdFilesDirectoryAction {
    std::string path;
  };

  CloudDataSources();

  std::map<Key, std::shared_ptr<SequentialPcdPlayer>> sequential_pcd_players() const {
    return sequential_pcd_players_;
  };


private:
  std::map<Key, std::shared_ptr<SequentialPcdPlayer>> sequential_pcd_players_;

  void initializeConnections();
  void onPcdFilesDirectoryOpen(const OpenPcdFilesDirectoryAction &action);
};

}

#endif //CIPOINTCLOUDVIEWERAPP_CLOUDDATASOURCES_H
