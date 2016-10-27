//
// Created by Masayuki IZUMI on 9/29/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_DEVICEMANAGERWINDOW_H
#define CIPOINTCLOUDVIEWERAPP_DEVICEMANAGERWINDOW_H

#include "Configure.h"
#include "SavingVerticesWorker.h"
#include "Window.h"
#include "Skeleton.h"
#include "io/SensorDeviceManager.hpp"
#include "io/exporter/Exporter.h"

namespace view {
namespace window {

class DeviceManagerWindow : public Window {
public:
  DeviceManagerWindow(
    const std::string name,
    const int width,
    const int spacing,
    const ImGuiWindowFlags flags,
    const std::shared_ptr<Configure> &config,
    const std::shared_ptr<SavingVerticesWorker> &saving_vertices_worker,
#ifdef USE_NITE2
    const std::shared_ptr<io::exporter::Exporter<SkeletonsPtr>> &skeletons_exporter,
#endif
    const std::shared_ptr<io::SensorDeviceManager> &sensor_device_manager
  );


protected:
  void drawImpl() override;


private:
  const std::shared_ptr<Configure> config_;
  const std::shared_ptr<SavingVerticesWorker> saving_vertices_worker_;
#ifdef USE_NITE2
  const std::shared_ptr<io::exporter::Exporter<SkeletonsPtr>> skeletons_exporter_;
#endif
  const std::shared_ptr<io::SensorDeviceManager> sensor_device_manager_;

  std::string device_selected_;

  void drawControls();
  void drawDeviceTable();
  void drawSavingPCDWidget();
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_DEVICEMANAGERWINDOW_H
