//
// Created by Masayuki IZUMI on 7/18/16.
//

#include "view/AppGui.h"
#include "view/menu/FileMenu.h"
#include "view/menu/ViewMenu.h"
#include "view/menu/WindowMenu.h"
#include "view/window/AppearanceWindow.h"
#include "view/window/CameraWindow.h"
#include "view/window/CloudsWindow.h"
#include "view/window/DeviceManagerWindow.h"
#include "view/window/FiltersWindow.h"
#include "view/window/InformationWindow.h"
#include "view/window/PlayerWindow.h"

namespace view {

AppGui::AppGui(
  ci::app::AppBase *app,
  const std::shared_ptr<Clouds> &clouds,
  const std::shared_ptr<ViewParams> &view_params,
  const std::shared_ptr<Configure> &config,
  const std::shared_ptr<io::CloudDataSources> &cloud_data_sources,
  const std::shared_ptr<io::SensorDeviceManager> &sensor_device_manager,
  const std::shared_ptr<SavingVerticesWorker> &saving_vertices_worker,
  const std::shared_ptr<io::exporter::SkeletonsExporter> &skeletons_exporter
)
  : app_(app)
  , view_params_            (view_params)
  , window_appearance_      (new view::window::AppearanceWindow(
    "Appearance", kWindowWidth, kWindowSpacing, kWindowFlags, view_params
  ))
  , window_camera_          (new view::window::CameraWindow(
    "Camera", kWindowWidth, kWindowSpacing, kWindowFlags, view_params
  ))
  , window_clouds_          (new view::window::CloudsWindow(
    "Clouds", kWindowWidth, kWindowSpacing, kWindowFlags, clouds
  ))
  , window_device_manager_  (new view::window::DeviceManagerWindow(
    "Connected devices", kWindowWidth, kWindowSpacing, kWindowFlags,
    config, saving_vertices_worker, skeletons_exporter, sensor_device_manager
  ))
  , window_filters_         (new view::window::FiltersWindow(
    "Filters", kWindowWidth, kWindowSpacing, kWindowFlags, clouds
  ))
  , window_info_            (new view::window::InformationWindow(
    "Information", kWindowWidth, kWindowSpacing, kWindowFlags, app, clouds
  ))
  , window_player_          (new view::window::PlayerWindow(
    "Player", kWindowWidth, kWindowSpacing, kWindowFlags, cloud_data_sources
  ))
  , menu_file_              (new view::menu::FileMenu("File", app, config))
  , menu_view_              (new view::menu::ViewMenu("View", config, view_params))
  , menu_window_            (new view::menu::WindowMenu(
    "Window",
    view_params,
    {
      { window_camera_, window_appearance_, window_filters_, window_clouds_, window_info_ },
      { window_player_, window_device_manager_ }
    }
  ))
{}

void AppGui::initialize() {
  ui::initialize(kUiOptions);
}

void AppGui::update() {
  auto left_window_pos = glm::vec2(kWindowSpacing, kWindowSpacing);
  auto right_window_pos = glm::vec2(app_->getWindowWidth() - (kWindowWidth + kWindowSpacing), kWindowSpacing);

  drawMenuBar(left_window_pos, right_window_pos);
  if (view_params_->is_windows_visible()) {
    drawWindows(left_window_pos, right_window_pos);
  }
}

void AppGui::drawMenuBar(glm::vec2 &left_window_pos, glm::vec2 &right_window_pos) {
  ui::ScopedMainMenuBar menu_bar;

  menu_file_->draw();
  menu_view_->draw();
  menu_window_->draw();

  left_window_pos.y += ui::GetItemRectSize().y;
  right_window_pos.y += ui::GetItemRectSize().y;
}

void AppGui::drawWindows(glm::vec2 &left_window_pos, glm::vec2 &right_window_pos) {
  window_camera_->draw(left_window_pos);
  window_appearance_->draw(left_window_pos);
  window_filters_->draw(left_window_pos);
  window_clouds_->draw(left_window_pos);
  window_info_->draw(left_window_pos);

  window_player_->draw(right_window_pos);
  window_device_manager_->draw(right_window_pos);
}

}
