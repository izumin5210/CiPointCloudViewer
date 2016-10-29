//
// Created by Masayuki IZUMI on 9/29/16.
//

#include <chrono>
#include "view/window/DeviceManagerWindow.h"
#include "util/util.h"

namespace view {
namespace window {

DeviceManagerWindow::DeviceManagerWindow(
  const std::string name,
  const int width,
  const int spacing,
  const ImGuiWindowFlags flags,
  const std::shared_ptr<Configure> &config,
  const std::shared_ptr<io::exporter::Exporters> &exporters,
  const std::shared_ptr<io::SensorDeviceManager> &sensor_device_manager
)
  : Window(name, width, spacing, flags)
  , config_(config)
  , exporters_(exporters)
  , sensor_device_manager_(sensor_device_manager)
  , device_selected_(std::string())
{}

void DeviceManagerWindow::drawImpl() {
  drawControls();
  drawSpacer();
  drawDeviceTable();
  drawSpacer();
  drawSavingPCDWidget();
}

void DeviceManagerWindow::drawControls() {
  if (ui::Button("Start All")) {
    for (auto pair : sensor_device_manager_->devices()) {
      // TODO: Disable if CalibrationParams does not exists.
      if (!pair.second->isRunning()) {
        pair.second->start();
      }
    }
  }
  ui::SameLine();
  if (ui::Button("Stop All")) {
    for (auto pair : sensor_device_manager_->devices()) {
      if (pair.second->isRunning()) {
        pair.second->stop();
      }
    }
  }

  if (ui::Button("Record All")) {
    for (auto pair : sensor_device_manager_->devices()) {
      if (pair.second->isRunning() && !pair.second->isRecording()) {
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
    if (device->isRunning()) {
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
}

void DeviceManagerWindow::drawDeviceTable() {
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
    ui::Text("%s", pair.second->state().c_str());
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

  drawSpacer();

  if (ui::Button("Refresh list")) {
    sensor_device_manager_->refresh();
  }
}

void DeviceManagerWindow::drawSavingPCDWidget() {
  bool is_recording = !exporters_->hasStopped();
  if (ui::Checkbox("Save point clouds and skeletons", &is_recording)) {
    if (is_recording) {
      boost::filesystem::path dir(config_->getSavePcdFilesTo());
      auto path = (dir / std::to_string(util::to_us(util::now()))).string();
      util::mkdir_p(path);
      exporters_->start(path);
    } else {
      exporters_->stopSafety();
    }
  }
  {
    ui::Text("Point clouds");
    auto total_count = exporters_->vertices_exporter()->total_size();
    auto saved_count = total_count - exporters_->vertices_exporter()->size();
    ui::LabelText("Saved files", "%zu / %zu", saved_count, total_count);
    ui::LabelText("Worker FPS", "%f", exporters_->vertices_exporter()->fps());
  }
#ifdef USE_NITE2
  {
    ui::Text("Skeletons");
    auto total_count = exporters_->skeletons_exporter()->total_size();
    auto saved_count = total_count - exporters_->skeletons_exporter()->size();
    ui::LabelText("Saved files", "%zu / %zu", saved_count, total_count);
    ui::LabelText("Worker FPS", "%f", exporters_->skeletons_exporter()->fps());
  }
#endif
}

}
}
