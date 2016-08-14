//
// Created by Masayuki IZUMI on 7/18/16.
//

#include "AppGui.h"
#include "Signal.h"

AppGui::AppGui(
  const std::shared_ptr<Clouds> &clouds,
  const std::shared_ptr<ViewParams> &view_params,
  const std::shared_ptr<Configure> &config,
  const std::shared_ptr<io::CloudDataSources> &cloud_data_sources,
  const std::shared_ptr<io::SensorDeviceManager> &sensor_device_manager,
  const std::shared_ptr<SavingVerticesWorker> &saving_vertices_worker
)
  : clouds_                     (clouds)
  , view_params_                (view_params)
  , config_                     (config)
  , cloud_data_sources_         (cloud_data_sources)
  , saving_vertices_worker_     (saving_vertices_worker)
  , visible_camera_window_      (true)
  , visible_appearance_window_  (true)
  , visible_filters_window_     (true)
  , visible_clouds_window_      (true)
  , visible_info_window_        (true)
  , visible_player_window_      (true)
  , visible_devices_window_     (true)
  , cloud_selected_             (std::string())
  , sensor_device_manager_      (sensor_device_manager)
  , device_selected_            (std::string())
{}

void AppGui::initialize() {
  ui::initialize(kUiOptions);
}

void AppGui::update(ci::app::AppBase *app) {
  auto left_window_pos = glm::vec2(kWindowSpacing, kWindowSpacing);
  auto right_window_pos = glm::vec2(app->getWindowWidth() - (kWindowWidth + kWindowSpacing), kWindowSpacing);
  drawMenuBar(app, left_window_pos, right_window_pos);
  if (visible_camera_window_) {
    drawCameraWindow(left_window_pos);
  }
  if (visible_appearance_window_) {
    drawAppearanceWindow(left_window_pos);
  }
  if (visible_filters_window_) {
    drawFiltersWindow(left_window_pos);
  }
  if (visible_clouds_window_) {
    drawCloudsWindow(left_window_pos);
  }
  if (visible_info_window_) {
    drawInfoWindow(app, left_window_pos);
  }
  if (visible_player_window_) {
    drawPlayerWindow(right_window_pos);
  }
  if (visible_devices_window_) {
    drawDevicesWindow(right_window_pos);
  }
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
    if (ui::MenuItem("Show grid", nullptr, view_params_->is_visible_grid())) {
      Signal<ViewParams::ToggleGridVisibilityAction>::emit({!view_params_->is_visible_grid()});
    }
    if (ui::MenuItem("Full screen", nullptr, view_params_->is_full_screen())) {
      Signal<ViewParams::ToggleFullScreenAction>::emit({!view_params_->is_full_screen()});
    }
    ui::EndMenu();
  }

  if (ui::BeginMenu("Window")) {
    ui::MenuItem("Camera",            nullptr, &visible_camera_window_);
    ui::MenuItem("Appearance",        nullptr, &visible_appearance_window_);
    ui::MenuItem("Filter",            nullptr, &visible_filters_window_);
    ui::MenuItem("Clouds",            nullptr, &visible_clouds_window_);
    ui::MenuItem("Information",       nullptr, &visible_info_window_);
    ui::Separator();
    ui::MenuItem("Player",            nullptr, &visible_player_window_);
    ui::MenuItem("Connected devices", nullptr, &visible_devices_window_);
    ui::EndMenu();
  }

  left_window_pos.y += ui::GetItemRectSize().y;
  right_window_pos.y += ui::GetItemRectSize().y;
}

void AppGui::drawCameraWindow(glm::vec2 &window_pos) {
  ui::ScopedWindow window("Camera", kWindowFlags);
  auto look_at = view_params_->look_at();
  if (ui::DragFloat3("Look at", &look_at[0])) {
    Signal<ViewParams::UpdateCameraParamsAction>::emit({view_params_->eye_point(), look_at});
  }
  auto eye_point = view_params_->eye_point();
  if (ui::DragFloat3("Eye point", &eye_point[0])) {
    Signal<ViewParams::UpdateCameraParamsAction>::emit({eye_point, view_params_->look_at()});
  }

  ui::SetWindowPos(window_pos);
  ui::SetWindowSize(glm::vec2(kWindowWidth, 0));
  window_pos.y += ui::GetWindowHeight() + kWindowSpacing;
}

void AppGui::drawAppearanceWindow(glm::vec2 &window_pos) {
  ui::ScopedWindow window("Appearance", kWindowFlags);
  float point_size = view_params_->point_size();
  if (ui::InputFloat("Point size", &point_size, 0.1f)) {
    Signal<ViewParams::UpdatePointSizeAction>::emit({point_size});
  }
  ci::Color bg_color = view_params_->bg_color();
  if (ui::ColorEdit3("Background", &bg_color[0])) {
    Signal<ViewParams::UpdateBgColorAction>::emit({bg_color});
  }
  ui::SetWindowPos(window_pos);
  ui::SetWindowSize(glm::vec2(kWindowWidth, 0));
  window_pos.y += ui::GetWindowHeight() + kWindowSpacing;
}

void AppGui::drawFiltersWindow(glm::vec2 &window_pos) {
  ui::ScopedWindow window("Filters", kWindowFlags);
  int id = 0;
  auto addFilter = [&](const char *label, std::function<void()> definition) {
    if (ui::TreeNode(label)) {
      ui::Columns(2);
      definition();
      ui::TreePop();
      ui::Columns(1);
    }
  };
  auto addUi = [&](const char *label, std::function<bool()> updater) -> bool {
    ui::PushID(id++);
    ui::AlignFirstTextHeightToWidgets();
    ui::Bullet();
    ui::TextUnformatted(label);
    ui::NextColumn();
    ui::PushItemWidth(-1);
    bool updated = updater();
    ui::PopItemWidth();
    ui::NextColumn();
    ui::PopID();
    return updated;
  };
  addFilter("Pass-through X", [&](){
    auto params = clouds_->x_pass_through_filter_params();
    bool updated = false;
    updated = updated || addUi("Enable", [&](){
      return ui::Checkbox("##value", &params.enable);
    });
    updated = updated || addUi("Range", [&](){
      return ui::DragFloatRange2("##value", &params.min, &params.max, 0.05f);
    });
    if (updated) {
      Signal<Clouds::UpdatePassThroughFilterParamsAction>::emit({"x", params});
    }
  });
  addFilter("Pass-through Y", [&](){
    auto params = clouds_->y_pass_through_filter_params();
    bool updated = false;
    updated = updated || addUi("Enable", [&](){
      return ui::Checkbox("##value", &params.enable);
    });
    updated = updated || addUi("Range", [&](){
      return ui::DragFloatRange2("##value", &params.min, &params.max, 0.05f);
    });
    if (updated) {
      Signal<Clouds::UpdatePassThroughFilterParamsAction>::emit({"y", params});
    }
  });
  addFilter("Pass-through Z", [&](){
    auto params = clouds_->z_pass_through_filter_params();
    bool updated = false;
    updated = updated || addUi("Enable", [&](){
      return ui::Checkbox("##value", &params.enable);
    });
    updated = updated || addUi("Range", [&](){
      return ui::DragFloatRange2("##value", &params.min, &params.max, 0.05f);
    });
    if (updated) {
      Signal<Clouds::UpdatePassThroughFilterParamsAction>::emit({"z", params});
    }
  });
  addFilter("Voxel filter", [&](){
    auto params = clouds_->voxel_filter_params();
    bool updated = false;
    updated = updated || addUi("Enable", [&](){
      return ui::Checkbox("##value", &params.enable);
    });
    updated = updated || addUi("Voxel size", [&](){
      return ui::DragFloat("##value", &params.size, 0.001f, 0.0f);
    });
    if (updated) {
      Signal<Clouds::UpdateVoxelFilterParamsAction>::emit({params});
    }
  });
  addFilter("Statistical outlier removal", [&](){
    auto params = clouds_->sor_filter_params();
    bool updated = false;
    updated = updated || addUi("Enable", [&](){
      return ui::Checkbox("##value", &params.enable);
    });
    updated = updated || addUi("MeanK", [&](){
      return ui::InputInt("##value", &params.mean_k, 1);
    });
    updated = updated || addUi("StddevMulThresh", [&](){
      return ui::InputFloat("##value", &params.stddev_mul_threshold, 0.1f);
    });
    if (updated) {
      Signal<Clouds::UpdateStatisticalOutlierRemovalFilterParamsAction>::emit({params});
    }
  });

  ui::SetWindowPos(window_pos);
  ui::SetWindowSize(glm::vec2(kWindowWidth, 0));
  window_pos.y += ui::GetWindowHeight() + kWindowSpacing;
}

void AppGui::drawCloudsWindow(glm::vec2 &window_pos) {
  ui::ScopedWindow window("Clouds", kWindowFlags);

  if (ui::Button("Clear")) {
    Signal<Clouds::ClearCloudsAction>::emit({});
    cloud_selected_ = std::string();
  }

  if (!cloud_selected_.empty()) {
    ui::SameLine();
    if (ui::Button("Remove")) {
      Signal<Clouds::RemoveCloudAction>::emit({cloud_selected_});
      cloud_selected_ = std::string();
    }

    ui::SameLine();
    bool visible = clouds_->clouds()[cloud_selected_]->is_visible();
    if (ui::Button(visible ? "Hide" : "Show")) {
        Signal<Clouds::ChangeCloudVisibilityAction>::emit({cloud_selected_, !visible});
    }
  }

  ui::ListBoxHeader("");
  clouds_->lock();
  for (auto pair : clouds_->clouds()) {
    if (ui::Selectable(pair.first.c_str(), !cloud_selected_.empty() && (cloud_selected_ == pair.first))) {
      cloud_selected_ = pair.first;
    }
  }
  clouds_->unlock();
  ui::ListBoxFooter();

  ui::SetWindowPos(window_pos);
  ui::SetWindowSize(glm::vec2(kWindowWidth, 0));
  window_pos.y += ui::GetWindowHeight() + kWindowSpacing;
}

void AppGui::drawInfoWindow(ci::app::AppBase *app, glm::vec2 &window_pos) {
  ui::ScopedWindow window("Information", kWindowFlags);
  ui::LabelText("FPS", "%f", app->getAverageFps());
  ui::LabelText("points", "%zu", clouds_->size());

  ui::SetWindowPos(window_pos);
  ui::SetWindowSize(glm::vec2(kWindowWidth, 0));
  window_pos.y += ui::GetWindowHeight() + kWindowSpacing;
}

void AppGui::drawPlayerWindow(glm::vec2 &window_pos) {
  ui::ScopedWindow window("Player", kWindowFlags);

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

  ui::SetWindowPos(window_pos);
  ui::SetWindowSize(glm::vec2(kWindowWidth, 0));
  window_pos.y += ui::GetWindowHeight() + kWindowSpacing;
}

void AppGui::drawDevicesWindow(glm::vec2 &window_pos) {
  ui::ScopedWindow window("Connected devices", kWindowFlags);

  if (ui::Button("Start All")) {
    for (auto pair : sensor_device_manager_->devices()) {
      if (pair.second->isReady()) {
        pair.second->start();
      }
    }
  }
  ui::SameLine();
  if (ui::Button("Stop All")) {
    for (auto pair : sensor_device_manager_->devices()) {
      if (pair.second->hasStarted()) {
        pair.second->stop();
      }
    }
  }

  if (ui::Button("Record All")) {
    for (auto pair : sensor_device_manager_->devices()) {
      if (!pair.second->isRecording()) {
        pair.second->record(config_->getSaveOniFilesTo());
      }
    }
  }
  ui::SameLine();
  if (ui::Button("Stop recording all")) {
    for (auto pair : sensor_device_manager_->devices()) {
      if (pair.second->isRecording()) {
        pair.second->stopRecording();
      }
    }
  }

  if (!device_selected_.empty()) {
    ui::Separator();
    auto device = sensor_device_manager_->devices()[device_selected_];
    if (device->hasStarted()) {
      if (ui::Button("Stop")) {
        device->stop();
      }
      ui::SameLine();
      if (device->isRecording()) {
        if (ui::Button("Stop recording")) {
          device->stopRecording();
        }
      } else if (ui::Button("Start recording")) {
        device->record(config_->getSaveOniFilesTo());
      }
    } else if (ui::Button("Start")) {
      device->start();
    }
  }

  ui::Dummy(glm::vec2(kWindowSpacing, kWindowSpacing));

  ui::Separator();
  ui::Columns(3, "Connected devices", true);
  ui::Text("serial");
  ui::NextColumn();
  ui::Text("state");
  ui::NextColumn();
  ui::Text("fps");
  ui::NextColumn();
  ui::Separator();
  for (auto pair : sensor_device_manager_->devices()) {
    auto selected = !device_selected_.empty() && (device_selected_ == pair.first);
    if (ui::Selectable(pair.second->serial().c_str(), selected, ImGuiSelectableFlags_SpanAllColumns)) {
      device_selected_ = pair.first;
    }
    ui::NextColumn();
    ui::Text("%s", pair.second->stateString().c_str());
    ui::NextColumn();
    ui::Text("%f", pair.second->fps());
    ui::NextColumn();
  }
  if (sensor_device_manager_->devices().empty()) {
    ui::Text("...");
    ui::NextColumn();
    ui::Text("NO DEVICES");
    ui::NextColumn();
    ui::Text("0");
    ui::NextColumn();
  }
  ui::Separator();
  ui::Columns(1);

  ui::Dummy(glm::vec2(kWindowSpacing, kWindowSpacing));

  if (ui::Button("Refresh list")) {
    sensor_device_manager_->refresh();
  }

  bool has_recording_pcd_files = !saving_vertices_worker_->has_stopped();
  if (ui::Checkbox("Save as PCD files", &has_recording_pcd_files)) {
    if (has_recording_pcd_files) {
      saving_vertices_worker_->start(config_->getSavePcdFilesTo());
    } else {
      saving_vertices_worker_->stop();
    }
  }
  auto total_count = saving_vertices_worker_->total_size();
  auto saved_count = total_count - saving_vertices_worker_->size();
  ui::LabelText("Saved files", "%zu / %zu", saved_count, total_count);

  ui::SetWindowPos(window_pos);
  ui::SetWindowSize(glm::vec2(kWindowWidth, 0));
  window_pos.y += ui::GetWindowHeight() + kWindowSpacing;
}
