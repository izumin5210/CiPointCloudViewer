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

// TODO: will move
#include "grabber/PcdGrabber.hpp"
#include "grabber/SequentialPcdGrabber.hpp"

#include "io/CalibrationParamsManager.h"
#include "io/SensorDeviceManager.hpp"

class AppGui {
public:
  using path = boost::filesystem::path;

  AppGui(
    const std::shared_ptr<Clouds> &clouds,
    const std::shared_ptr<ViewParams> &view_params,
    const std::shared_ptr<Configure> &config,
    const std::shared_ptr<io::SensorDeviceManager> &sensor_device_manager_
  );

  void initialize();
  void update(ci::app::AppBase *app);


private:
  const ImGuiWindowFlags kWindowFlags = ImGuiWindowFlags_ShowBorders;
  const int kWindowSpacing = 8;
  const int kWindowWidth = 320;

  const ci::ColorA8u kColorBlackA55   = ci::ColorA8u(0x22, 0x22, 0x22, 0x55);
  const ci::ColorA8u kColorBlackAcc   = ci::ColorA8u(0x22, 0x22, 0x22, 0xcc);
  // bleu de provence
  const ci::ColorA8u kColorPrimary    = ci::ColorA8u(0x00, 0x9a, 0xc5, 0xcc);
  const ci::ColorA8u kColorPrimaryA99 = ci::ColorA8u(0x00, 0x9a, 0xc5, 0x99);
  const ci::ColorA8u kColorPrimaryA22 = ci::ColorA8u(0x00, 0x9a, 0xc5, 0x22);
  // rosso di toscana
  const ci::ColorA8u kColorAccent     = ci::ColorA8u(0xf1, 0x67, 0x3f, 0xee);
  const ci::ColorA8u kColorAccentAcc  = ci::ColorA8u(0xf1, 0x67, 0x3f, 0xcc);
  const ci::ColorA8u kColorAccentA99  = ci::ColorA8u(0xf1, 0x67, 0x3f, 0x99);

  const ui::Options kUiOptions = ui::Options()
    .darkTheme()
    .color(ImGuiCol_MenuBarBg,              kColorPrimaryA22)
    .color(ImGuiCol_TitleBg,                kColorPrimaryA22)
    .color(ImGuiCol_TitleBgCollapsed,       kColorPrimaryA22)
    .color(ImGuiCol_TitleBgActive,          kColorPrimaryA99)
    .color(ImGuiCol_WindowBg,               kColorPrimaryA22)
    .color(ImGuiCol_Border,                 kColorPrimaryA99)
    .color(ImGuiCol_FrameBg,                kColorPrimaryA22)
    .color(ImGuiCol_FrameBgHovered,         kColorAccentAcc)
    .color(ImGuiCol_FrameBgActive,          kColorAccent)
    .color(ImGuiCol_ScrollbarBg,            kColorPrimaryA22)
    .color(ImGuiCol_ScrollbarGrab,          kColorPrimaryA99)
    .color(ImGuiCol_ScrollbarGrabHovered,   kColorPrimaryA99)
    .color(ImGuiCol_ScrollbarGrabActive,    kColorPrimary)
    .color(ImGuiCol_CheckMark,              kColorAccent)
    .color(ImGuiCol_SliderGrab,             kColorPrimaryA99)
    .color(ImGuiCol_SliderGrabActive,       kColorPrimary)
    .color(ImGuiCol_Button,                 kColorPrimaryA22)
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
    .color(ImGuiCol_PopupBg,                kColorBlackAcc)
    .antiAliasedLines(true)
    .antiAliasedShapes(true)
    .windowRounding(0.0f)
    .frameRounding(0.0f);

  const std::shared_ptr<Clouds> clouds_;
  const std::shared_ptr<ViewParams> view_params_;
  const std::shared_ptr<Configure> config_;

  bool visible_camera_window_     = true;
  bool visible_appearance_window_ = true;
  bool visible_filters_window_    = true;
  bool visible_clouds_window_     = true;
  bool visible_info_window_       = true;
  bool visible_player_window_     = true;
  bool visible_devices_window_    = true;

  Clouds::Key cloud_selected_;

  // TODO: will move
  std::map<boost::filesystem::path, std::shared_ptr<grabber::PointCloudGrabber>> grabbers_;
  std::shared_ptr<grabber::PointCloudGrabber> grabber_selected_;
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
