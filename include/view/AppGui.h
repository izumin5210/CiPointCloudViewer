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
#include "Skeleton.h"

#include "io/CloudDataSources.h"
#include "io/CapturedLogManager.h"
#include "io/SensorDeviceManager.hpp"
#include "io/exporter/Exporters.h"

#include "menu/Menu.h"
#include "window/Window.h"

namespace view {

class AppGui {
public:
  AppGui(
    ci::app::AppBase *app,
    const std::shared_ptr<Clouds> &clouds,
    const std::shared_ptr<ViewParams> &view_params,
    const std::shared_ptr<Configure> &config,
    const std::shared_ptr<io::CloudDataSources> &cloud_data_sources,
    const std::shared_ptr<io::CapturedLogManager> &captured_log_manager,
    const std::shared_ptr<io::SensorDeviceManager> &sensor_device_manager,
    const std::shared_ptr<io::exporter::Exporters> &exporters
  );

  void initialize();
  void update();


private:
  const ImGuiWindowFlags kWindowFlags = ImGuiWindowFlags_ShowBorders;
  const int kWindowSpacing = 8;
  const int kWindowWidth = 320;

  const ci::Color8u kColorPrimaryBase = ci::Color8u(0x1c, 0xfe, 0xff);
  const ci::ColorA8u kColorPrimary    = ci::ColorA8u(kColorPrimaryBase, 0xcc);
  const ci::ColorA8u kColorPrimaryA99 = ci::ColorA8u(kColorPrimaryBase, 0x99);
  const ci::ColorA8u kColorPrimaryA66 = ci::ColorA8u(kColorPrimaryBase, 0x66);
  const ci::ColorA8u kColorPrimaryA33 = ci::ColorA8u(kColorPrimaryBase, 0x33);
  const ci::ColorA8u kColorPrimaryDark = ci::ColorA8u(0x03, 0x19, 0x1a, 0xbb);

  const ci::Color8u kColorAccentBase  = ci::Color8u(0xff, 0xa1, 0x00);
  const ci::ColorA8u kColorAccent     = ci::ColorA8u(kColorAccentBase, 0xee);
  const ci::ColorA8u kColorAccentAcc  = ci::ColorA8u(kColorAccentBase, 0xcc);
  const ci::ColorA8u kColorAccentA99  = ci::ColorA8u(kColorAccentBase, 0x99);

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
    .color(ImGuiCol_TextDisabled,           kColorPrimaryA66)
    .color(ImGuiCol_TextSelectedBg,         kColorAccent)
    .color(ImGuiCol_PopupBg,                kColorPrimaryDark)
    .antiAliasedLines(true)
    .antiAliasedShapes(true)
    .windowRounding(0.0f)
    .frameRounding(0.0f);

  ci::app::AppBase *app_;

  const std::shared_ptr<ViewParams> view_params_;

  std::shared_ptr<view::window::Window>
    window_appearance_,
    window_camera_,
    window_clouds_,
    window_device_manager_,
    window_filters_,
    window_info_,
    window_player_;

  std::shared_ptr<view::menu::Menu>
    menu_file_,
    menu_view_,
    menu_window_;


  void drawMenuBar(glm::vec2 &left_window_pos, glm::vec2 &right_window_pos);
  void drawWindows(glm::vec2 &left_window_pos, glm::vec2 &right_window_pos);
};

}
#endif //CIPOINTCLOUDVIEWERAPP_APPGUI_H
