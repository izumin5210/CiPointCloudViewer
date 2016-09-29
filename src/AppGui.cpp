//
// Created by Masayuki IZUMI on 7/18/16.
//

#include "AppGui.h"
#include "Signal.h"

AppGui::AppGui(
  ci::app::AppBase *app,
  const std::shared_ptr<Clouds> &clouds,
  const std::shared_ptr<ViewParams> &view_params,
  const std::shared_ptr<Configure> &config,
  const std::shared_ptr<io::CloudDataSources> &cloud_data_sources,
  const std::shared_ptr<io::SensorDeviceManager> &sensor_device_manager,
  const std::shared_ptr<SavingVerticesWorker> &saving_vertices_worker
)
  : clouds_                 (clouds)
  , view_params_            (view_params)
  , config_                 (config)
  , cloud_data_sources_     (cloud_data_sources)
  , saving_vertices_worker_ (saving_vertices_worker)
  , sensor_device_manager_  (sensor_device_manager)
  , appearance_window_      (new view::window::AppearanceWindow(
    "Appearance", kWindowWidth, kWindowSpacing, kWindowFlags, view_params
  ))
  , camera_window_          (new view::window::CameraWindow(
    "Camera", kWindowWidth, kWindowSpacing, kWindowFlags, view_params
  ))
  , clouds_window_          (new view::window::CloudsWindow(
    "Clouds", kWindowWidth, kWindowSpacing, kWindowFlags, clouds
  ))
  , device_manager_window_  (new view::window::DeviceManagerWindow(
    "Connected devices", kWindowWidth, kWindowSpacing, kWindowFlags,
    config, saving_vertices_worker, sensor_device_manager
  ))
  , filters_window_         (new view::window::FiltersWindow(
    "Filters", kWindowWidth, kWindowSpacing, kWindowFlags, clouds
  ))
  , info_window_            (new view::window::InformationWindow(
    "Information", kWindowWidth, kWindowSpacing, kWindowFlags, app, clouds
  ))
  , player_window_          (new view::window::PlayerWindow(
    "Player", kWindowWidth, kWindowSpacing, kWindowFlags, cloud_data_sources
  ))
  , file_menu_              (new view::menu::FileMenu("File", app, config))
  , view_menu_              (new view::menu::ViewMenu("View", config, view_params))
  , window_menu_            (new view::menu::WindowMenu(
    "Window",
    {
      { camera_window_, appearance_window_, filters_window_, clouds_window_, info_window_ },
      { player_window_, device_manager_window_ }
    }
  ))
{}

void AppGui::initialize() {
  ui::initialize(kUiOptions);
}

void AppGui::update(ci::app::AppBase *app) {
  auto left_window_pos = glm::vec2(kWindowSpacing, kWindowSpacing);
  auto right_window_pos = glm::vec2(app->getWindowWidth() - (kWindowWidth + kWindowSpacing), kWindowSpacing);
  drawMenuBar(left_window_pos, right_window_pos);

  camera_window_->draw(left_window_pos);
  appearance_window_->draw(left_window_pos);
  filters_window_->draw(left_window_pos);
  clouds_window_->draw(left_window_pos);
  info_window_->draw(left_window_pos);

  player_window_->draw(right_window_pos);
  device_manager_window_->draw(right_window_pos);
}

void AppGui::drawMenuBar(glm::vec2 &left_window_pos, glm::vec2 &right_window_pos) {
  ui::ScopedMainMenuBar menu_bar;

  file_menu_->draw();
  view_menu_->draw();
  window_menu_->draw();

  left_window_pos.y += ui::GetItemRectSize().y;
  right_window_pos.y += ui::GetItemRectSize().y;
}
