//
// Created by Masayuki IZUMI on 7/18/16.
//

#include "CinderImGui.h"

#include "impl/AppGuiImpl.h"
#include "action/CloudDataSourcesAction.h"
#include "action/ViewParamsAction.h"

class AppGuiImpl : public AppGui {
public:
  using path = boost::filesystem::path;

  INJECT(AppGuiImpl(
      std::shared_ptr<Dispatcher> dispatcher,
      std::shared_ptr<Clouds> clouds,
      std::shared_ptr<ViewParams> view_params,
      std::shared_ptr<Configure> config,
      std::shared_ptr<io::SensorDeviceManager> sensor_device_manager,
      std::shared_ptr<io::CloudDataSources> cloud_data_sources,
      std::shared_ptr<SavingVerticesWorker> saving_vertices_worker
  ))
    : dispatcher_                 (dispatcher)
    , clouds_                     (clouds)
    , view_params_                (view_params)
    , config_                     (config)
    , sensor_device_manager_      (sensor_device_manager)
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
    , device_selected_            (std::string())
  {}

  void initialize() override {
    ui::initialize(kUiOptions);
  }

  void update(ci::app::AppBase *app) override {
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


private:
  const ImGuiWindowFlags kWindowFlags = ImGuiWindowFlags_ShowBorders;
  const int kWindowSpacing = 8;
  const int kWindowWidth = 320;

  // bleu de provence
  const ci::ColorA8u kColorPrimary    = ci::ColorA8u(0x00, 0x9a, 0xc5, 0xcc);
  const ci::ColorA8u kColorPrimaryA99 = ci::ColorA8u(0x00, 0x9a, 0xc5, 0x99);
  const ci::ColorA8u kColorPrimaryA33 = ci::ColorA8u(0x00, 0x9a, 0xc5, 0x33);
  const ci::ColorA8u kColorPrimaryDark = ci::ColorA8u(0x00, 0x27, 0x33, 0xbb);
  // rosso di toscana
  const ci::ColorA8u kColorAccent     = ci::ColorA8u(0xf1, 0x67, 0x3f, 0xee);
  const ci::ColorA8u kColorAccentAcc  = ci::ColorA8u(0xf1, 0x67, 0x3f, 0xcc);
  const ci::ColorA8u kColorAccentA99  = ci::ColorA8u(0xf1, 0x67, 0x3f, 0x99);

  const ci::ColorA8u kColorWhite      = ci::ColorA8u(0xdd, 0xdd, 0xdd, 0xcc);
  const ci::ColorA8u kColorBlackA55   = ci::ColorA8u(0x11, 0x11, 0x11, 0x55);

  const ui::Options kUiOptions = ui::Options()
      .darkTheme()
      .color(ImGuiCol_MenuBarBg,              kColorPrimaryA33)
      .color(ImGuiCol_TitleBg,                kColorPrimaryDark)
      .color(ImGuiCol_TitleBgCollapsed,       kColorPrimaryDark)
      .color(ImGuiCol_TitleBgActive,          kColorPrimaryA99)
      .color(ImGuiCol_WindowBg,               kColorPrimaryDark)
      .color(ImGuiCol_Border,                 kColorPrimaryA99)
      .color(ImGuiCol_FrameBg,                kColorPrimaryA33)
      .color(ImGuiCol_FrameBgHovered,         kColorAccentAcc)
      .color(ImGuiCol_FrameBgActive,          kColorAccent)
      .color(ImGuiCol_ScrollbarBg,            kColorPrimaryA33)
      .color(ImGuiCol_ScrollbarGrab,          kColorPrimaryA99)
      .color(ImGuiCol_ScrollbarGrabHovered,   kColorPrimaryA99)
      .color(ImGuiCol_ScrollbarGrabActive,    kColorPrimary)
      .color(ImGuiCol_CheckMark,              kColorAccent)
      .color(ImGuiCol_SliderGrab,             kColorPrimaryA99)
      .color(ImGuiCol_SliderGrabActive,       kColorPrimary)
      .color(ImGuiCol_Button,                 kColorPrimaryA33)
      .color(ImGuiCol_ButtonHovered,          kColorAccentAcc)
      .color(ImGuiCol_ButtonActive,           kColorAccent)
      .color(ImGuiCol_Header,                 kColorAccentA99)
      .color(ImGuiCol_HeaderHovered,          kColorAccentAcc)
      .color(ImGuiCol_HeaderActive,           kColorAccent)
      .color(ImGuiCol_Column,                 kColorBlackA55)
      .color(ImGuiCol_ColumnHovered,          kColorAccentAcc)
      .color(ImGuiCol_ColumnActive,           kColorAccent)
      .color(ImGuiCol_PlotLines,              kColorPrimaryA99)
      .color(ImGuiCol_PlotLinesHovered,       kColorPrimary)
      .color(ImGuiCol_PlotHistogram,          kColorPrimaryA99)
      .color(ImGuiCol_PlotHistogramHovered,   kColorPrimary)
      .color(ImGuiCol_Text,                   kColorPrimary)
      .color(ImGuiCol_TextDisabled,           kColorBlackA55)
      .color(ImGuiCol_TextSelectedBg,         kColorAccent)
      .color(ImGuiCol_PopupBg,                kColorPrimaryDark)
      .antiAliasedLines(true)
      .antiAliasedShapes(true)
      .windowRounding(0.0f)
      .frameRounding(0.0f);

  const std::shared_ptr<Dispatcher> dispatcher_;
  const std::shared_ptr<Clouds> clouds_;
  const std::shared_ptr<ViewParams> view_params_;
  const std::shared_ptr<Configure> config_;
  const std::shared_ptr<io::SensorDeviceManager> sensor_device_manager_;
  const std::shared_ptr<io::CloudDataSources> cloud_data_sources_;
  const std::shared_ptr<SavingVerticesWorker> saving_vertices_worker_;

  bool visible_camera_window_     = true;
  bool visible_appearance_window_ = true;
  bool visible_filters_window_    = true;
  bool visible_clouds_window_     = true;
  bool visible_info_window_       = true;
  bool visible_player_window_     = true;
  bool visible_devices_window_    = true;

  Cloud::Key cloud_selected_;

  std::string device_selected_;


  void drawMenuBar(ci::app::AppBase *app, glm::vec2 &left_window_pos, glm::vec2 &right_window_pos) {
    ui::ScopedMainMenuBar menu_bar;

    if (ui::BeginMenu("File")) {
      if (ui::MenuItem("Open *.pcd file")) {
        auto pcdfile = app->getOpenFilePath(path(), {"pcd"});
        if (boost::filesystem::exists(pcdfile)) {
          dispatcher_->emit<OpenPcdFileAction>({pcdfile.string()});
        }
      }
      if(ui::MenuItem("Open directory")) {
        auto dir = app->getFolderPath();
        if (boost::filesystem::is_directory(dir)) {
          dispatcher_->emit<OpenPcdFilesDirectoryAction>({dir.string()});
//        grabbers_[dir] = std::make_shared<grabber::SequentialPcdGrabber>(dir);
        }
      }
      if (ui::MenuItem("Open calibration yaml file")) {
        auto yamlfile = app->getOpenFilePath(boost::filesystem::path(), {"yaml", "yml"});
        if (boost::filesystem::exists(yamlfile)) {
          for (auto pair : io::CalibrationParams::load(yamlfile.string())) {
            dispatcher_->emit<UpdateCalibrationParamsAction>({ pair.first, pair.second });
          }
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
        dispatcher_->emit<ToggleFullScreenAction>({!view_params_->is_full_screen()});
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

  void drawCameraWindow(glm::vec2 &window_pos) {
    ui::ScopedWindow window("Camera", kWindowFlags);
    auto look_at = view_params_->look_at();
    if (ui::DragFloat3("Look at", &look_at[0])) {
      dispatcher_->emit<UpdateCameraParamsAction>({view_params_->eye_point(), look_at});
    }
    auto eye_point = view_params_->eye_point();
    if (ui::DragFloat3("Eye point", &eye_point[0])) {
      dispatcher_->emit<UpdateCameraParamsAction>({eye_point, view_params_->look_at()});
    }

    ui::SetWindowPos(window_pos);
    ui::SetWindowSize(glm::vec2(kWindowWidth, 0));
    window_pos.y += ui::GetWindowHeight() + kWindowSpacing;
  }

  void drawAppearanceWindow(glm::vec2 &window_pos) {
    ui::ScopedWindow window("Appearance", kWindowFlags);
    float point_size = view_params_->point_size();
    if (ui::InputFloat("Point size", &point_size, 0.1f)) {
      dispatcher_->emit<UpdatePointSizeAction>({point_size});
    }
    ci::Color bg_color = view_params_->bg_color();
    if (ui::ColorEdit3("Background", &bg_color[0])) {
      dispatcher_->emit<UpdateBgColorAction>({bg_color});
    }
    ui::SetWindowPos(window_pos);
    ui::SetWindowSize(glm::vec2(kWindowWidth, 0));
    window_pos.y += ui::GetWindowHeight() + kWindowSpacing;
  }

  void drawFiltersWindow(glm::vec2 &window_pos) {
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
        dispatcher_->emit<UpdatePassThroughFilterParamsAction>({"x", params});
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
        dispatcher_->emit<UpdatePassThroughFilterParamsAction>({"y", params});
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
        dispatcher_->emit<UpdatePassThroughFilterParamsAction>({"z", params});
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
        dispatcher_->emit<UpdateVoxelFilterParamsAction>({params});
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
        dispatcher_->emit<UpdateStatisticalOutlierRemovalFilterParamsAction>({params});
      }
    });

    ui::SetWindowPos(window_pos);
    ui::SetWindowSize(glm::vec2(kWindowWidth, 0));
    window_pos.y += ui::GetWindowHeight() + kWindowSpacing;
  }

  void drawCloudsWindow(glm::vec2 &window_pos) {
    ui::ScopedWindow window("Clouds", kWindowFlags);

    if (ui::Button("Clear")) {
      dispatcher_->emit<ClearCloudsAction>({});
      cloud_selected_ = std::string();
    }

    if (!cloud_selected_.empty()) {
      ui::SameLine();
      if (ui::Button("Remove")) {
        dispatcher_->emit<RemoveCloudAction>({cloud_selected_});
        cloud_selected_ = std::string();
      }

      ui::SameLine();
      bool visible = clouds_->clouds()[cloud_selected_]->is_visible();
      if (ui::Button(visible ? "Hide" : "Show")) {
        dispatcher_->emit<ChangeCloudVisibilityAction>({cloud_selected_, !visible});
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

  void drawInfoWindow(ci::app::AppBase *app, glm::vec2 &window_pos) {
    ui::ScopedWindow window("Information", kWindowFlags);
    ui::LabelText("FPS", "%f", app->getAverageFps());
    ui::LabelText("points", "%zu", clouds_->size());

    ui::SetWindowPos(window_pos);
    ui::SetWindowSize(glm::vec2(kWindowWidth, 0));
    window_pos.y += ui::GetWindowHeight() + kWindowSpacing;
  }

  void drawPlayerWindow(glm::vec2 &window_pos) {
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

  void drawDevicesWindow(glm::vec2 &window_pos) {
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
        saving_vertices_worker_->stopSafety();
      }
    }
    auto total_count = saving_vertices_worker_->total_size();
    auto saved_count = total_count - saving_vertices_worker_->size();
    ui::LabelText("Saved files", "%zu / %zu", saved_count, total_count);
    ui::LabelText("Worker FPS", "%f", saving_vertices_worker_->fps());

    ui::SetWindowPos(window_pos);
    ui::SetWindowSize(glm::vec2(kWindowWidth, 0));
    window_pos.y += ui::GetWindowHeight() + kWindowSpacing;
  }
};


fruit::Component<
    fruit::Required<
        Dispatcher,
        Clouds,
        ViewParams,
        Configure,
        io::SensorDeviceManager,
        io::CloudDataSources,
        SavingVerticesWorker
    >,
    AppGui
>
getAppGuiImplComponent() {
  return fruit::createComponent()
      .bind<AppGui, AppGuiImpl>();
};

