//
// Created by Masayuki IZUMI on 7/20/16.
//

#include "Signal.h"
#include "io/CloudDataSources.h"

namespace io {

CloudDataSources::CloudDataSources() {
  initializeConnections();
}

void CloudDataSources::initializeConnections() {
  Signal<OpenPcdFilesDirectoryAction>::connect(
    std::bind(&CloudDataSources::onPcdFilesDirectoryOpen, this, std::placeholders::_1)
  );
}

void CloudDataSources::onPcdFilesDirectoryOpen(const OpenPcdFilesDirectoryAction &action) {
  sequential_pcd_players_[action.path] = std::make_shared<SequentialPcdPlayer>(action.path);
}

}
