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
  virtual std::map<Key, std::shared_ptr<SequentialPcdPlayer>> sequential_pcd_players() const = 0;
};

}

#endif //CIPOINTCLOUDVIEWERAPP_CLOUDDATASOURCES_H
