//
// Created by Masayuki IZUMI on 9/29/16.
//

#include "view/window/PlayerWindow.h"

namespace view {
namespace window {

PlayerWindow::PlayerWindow(
  const std::string name,
  const int width,
  const int spacing,
  const ImGuiWindowFlags flags,
  const std::shared_ptr<io::CloudDataSources> &cloud_data_sources
)
  : Window(name, width, spacing, flags)
  , cloud_data_sources_(cloud_data_sources)
{}

void PlayerWindow::drawImpl() {
  for (auto pair : cloud_data_sources_->sequential_pcd_players()) {
    ui::TextUnformatted(boost::filesystem::path(pair.first).filename().c_str());
    auto player = pair.second;
    auto prg = player->loading_progress();
    ui::Columns(2);
    ui::ProgressBar(((float) prg[0]) / prg[1]);
    ui::NextColumn();
    ui::Text("%04d / %04d", (int) prg[0], (int) prg[1]);
    ui::SameLine();
    if (player->isPlaying()) {
      if (ui::Button("Stop")) {
        player->stop();
      }
    } else if (ui::Button("Play")) {
      player->start();
    }
    ui::Columns(1);
  }
}

}
}
