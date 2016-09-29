//
// Created by Masayuki IZUMI on 7/18/16.
//

#include "AppGui.h"
#include "Signal.h"

AppGui::AppGui(
  const ci::app::AppBase *app,
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
  , device_selected_        (std::string())
  , appearance_window_      ("Appearance", kWindowWidth, kWindowSpacing, kWindowFlags, view_params)
  , camera_window_          ("Camera", kWindowWidth, kWindowSpacing, kWindowFlags, view_params)
  , clouds_window_          ("Clouds", kWindowWidth, kWindowSpacing, kWindowFlags, clouds)
  , device_manager_window_  ("Connected devices", kWindowWidth, kWindowSpacing, kWindowFlags,
                             config, saving_vertices_worker, sensor_device_manager)
  , filters_window_         ("Filters", kWindowWidth, kWindowSpacing, kWindowFlags, clouds)
  , info_window_            ("Information", kWindowWidth, kWindowSpacing, kWindowFlags, app, clouds)
  , player_window_          ("Player", kWindowWidth, kWindowSpacing, kWindowFlags, cloud_data_sources)
{}

void AppGui::initialize() {
  ui::initialize(kUiOptions);
}

void AppGui::update(ci::app::AppBase *app) {
  auto left_window_pos = glm::vec2(kWindowSpacing, kWindowSpacing);
  auto right_window_pos = glm::vec2(app->getWindowWidth() - (kWindowWidth + kWindowSpacing), kWindowSpacing);
  drawMenuBar(app, left_window_pos, right_window_pos);

  camera_window_.draw(left_window_pos);
  appearance_window_.draw(left_window_pos);
  filters_window_.draw(left_window_pos);
  clouds_window_.draw(left_window_pos);
  info_window_.draw(left_window_pos);

  player_window_.draw(right_window_pos);
  device_manager_window_.draw(right_window_pos);
}

void AppGui::drawMenuBar(ci::app::AppBase *app, glm::vec2 &left_window_pos, glm::vec2 &right_window_pos) {
  ui::ScopedMainMenuBar menu_bar;

  if (ui::BeginMenu("File")) {
    if (ui::MenuItem("Open *.pcd file")) {
      auto pcdfile = app->getOpenFilePath(path(), {"pcd"});
      if (boost::filesystem::exists(pcdfile)) {
        Signal<Clouds::OpenPcdFileAction>::emit({pcdfile.string()});
      }
    }
    if(ui::MenuItem("Open directory")) {
      auto dir = app->getFolderPath();
      if (boost::filesystem::is_directory(dir)) {
        Signal<io::CloudDataSources::OpenPcdFilesDirectoryAction>::emit({dir.string()});
//        grabbers_[dir] = std::make_shared<grabber::SequentialPcdGrabber>(dir);
      }
    }
    if (ui::MenuItem("Open calibration yaml file")) {
      auto yamlfile = app->getOpenFilePath(boost::filesystem::path(), {"yaml", "yml"});
      if (boost::filesystem::exists(yamlfile)) {
        io::CalibrationParams::load(yamlfile.string());
      }
    }
    ui::Separator();
    if (ui::MenuItem("Save *.oni to ...")) {
      auto dir = app->getFolderPath(path(config_->getSaveOniFilesTo()));
      if (boost::filesystem::is_directory(dir)) {
        config_->setSaveOniFilesTo(dir.string());
      }
    }
    if (ui::MenuItem("Save *.pcd to ...")) {
      auto dir = app->getFolderPath(path(config_->getSavePcdFilesTo()));
      if (boost::filesystem::is_directory(dir)) {
        config_->setSavePcdFilesTo(dir.string());
      }
    }
    ui::EndMenu();
  }

  if (ui::BeginMenu("View")) {
    if (ui::BeginMenu("Grid")) {
      if (ui::MenuItem("None", nullptr, view_params_->grid() == ViewParams::Grid::NONE)) {
        config_->setSaveGridType(ViewParams::Grid::NONE);
      }
      if (ui::MenuItem("Rectangular grid", nullptr, view_params_->grid() == ViewParams::Grid::RECTANGULAR)) {
        config_->setSaveGridType(ViewParams::Grid::RECTANGULAR);
      }
      if (ui::MenuItem("Polar grid", nullptr, view_params_->grid() == ViewParams::Grid::POLAR)) {
        config_->setSaveGridType(ViewParams::Grid::POLAR);
      }
      ui::EndMenu();
    }
    ui::Separator();
    if (ui::MenuItem("Full screen", nullptr, view_params_->is_full_screen())) {
      Signal<ViewParams::ToggleFullScreenAction>::emit({!view_params_->is_full_screen()});
    }
    ui::EndMenu();
  }

  if (ui::BeginMenu("Window")) {
    drawToggleWindowVisibilityItem(camera_window_);
    drawToggleWindowVisibilityItem(appearance_window_);
    drawToggleWindowVisibilityItem(filters_window_);
    drawToggleWindowVisibilityItem(clouds_window_);
    drawToggleWindowVisibilityItem(info_window_);
    ui::Separator();
    drawToggleWindowVisibilityItem(player_window_);
    drawToggleWindowVisibilityItem(device_manager_window_);
    ui::EndMenu();
  }

  left_window_pos.y += ui::GetItemRectSize().y;
  right_window_pos.y += ui::GetItemRectSize().y;
}

void AppGui::drawToggleWindowVisibilityItem(view::window::Window &window) {
  if (ui::MenuItem(window.name().c_str(), nullptr, window.visible())) {
    window.toggleVisibility();
  }
}
