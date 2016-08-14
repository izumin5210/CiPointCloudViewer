//
// Created by Masayuki IZUMI on 7/18/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_APPGUI_H
#define CIPOINTCLOUDVIEWERAPP_APPGUI_H

#include "cinder/app/AppBase.h"
#include "CinderImGui.h"

#include "Clouds.h"
#include "ViewParams.h"
#include "Configure.h"
#include "SavingVerticesWorker.h"

#include "io/CloudDataSources.h"
#include "io/CalibrationParamsManager.h"
#include "io/SensorDeviceManager.hpp"

class AppGui {
public:
  using path = boost::filesystem::path;

  AppGui(
    const std::shared_ptr<Clouds> &clouds,
    const std::shared_ptr<ViewParams> &view_params,
    const std::shared_ptr<Configure> &config,
    const std::shared_ptr<io::CloudDataSources> &cloud_data_sources,
    const std::shared_ptr<io::SensorDeviceManager> &sensor_device_manager,
    const std::shared_ptr<SavingVerticesWorker> &saving_vertices_worker
  );

  void initialize();
  void update(ci::app::AppBase *app);


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

  const std::shared_ptr<Clouds> clouds_;
  const std::shared_ptr<ViewParams> view_params_;
  const std::shared_ptr<Configure> config_;
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

  std::shared_ptr<io::SensorDeviceManager> sensor_device_manager_;
  std::string device_selected_;


  void drawMenuBar(ci::app::AppBase *app, glm::vec2 &left_window_pos, glm::vec2 &right_window_pos);
  void drawCameraWindow(glm::vec2 &window_pos);
  void drawAppearanceWindow(glm::vec2 &window_pos);
  void drawFiltersWindow(glm::vec2 &window_pos);
  void drawCloudsWindow(glm::vec2 &window_pos);
  void drawInfoWindow(ci::app::AppBase *app, glm::vec2 &window_pos);
  void drawPlayerWindow(glm::vec2 &window_pos);
  void drawDevicesWindow(glm::vec2 &window_pos);
};

#endif //CIPOINTCLOUDVIEWERAPP_APPGUI_H
