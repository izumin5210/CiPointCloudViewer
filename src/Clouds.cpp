//
// Created by Masayuki IZUMI on 7/19/16.
//

#include <pcl/io/pcd_io.h>

#include "Clouds.h"
#include "Signal.h"

Clouds::Clouds()
  : x_pass_through_filter_("x")
  , y_pass_through_filter_("y")
  , z_pass_through_filter_("z")
#ifdef USE_NITE2
  , enable_users_through_filter_(false)
#endif
{
  initializeConnections();
}

void Clouds::initializeConnections() {
  namespace ph = std::placeholders;
  addConnection(Signal<UpdatePointsAction>::connect(std::bind(&Clouds::onPointsUpdate, this, ph::_1)));
  addConnection(Signal<UpdateVerticesAction>::connect(std::bind(&Clouds::onVerticesUpdate, this, ph::_1)));
  addConnection(Signal<UpdateCalibrationParamsAction>::connect(std::bind(&Clouds::onCalibrationParamsUpdate, this, ph::_1)));
  addConnection(Signal<ChangeCloudVisibilityAction>::connect(std::bind(&Clouds::onCloudVisibilityChange, this, ph::_1)));
  addConnection(Signal<RemoveCloudAction>::connect(std::bind(&Clouds::onCloudRemove, this, ph::_1)));
  addConnection(Signal<ClearCloudsAction>::connect(std::bind(&Clouds::onCloudsClear, this, ph::_1)));
  addConnection(Signal<UpdatePassThroughFilterParamsAction>::connect(
    std::bind(&Clouds::onPassThroughFilterParamsUpdate, this, ph::_1)));
#ifdef USE_NITE2
  addConnection(Signal<UpdateUsersThroughFitlerParamsAction>::connect(
    std::bind(&Clouds::onUsersThroughFitlerParamsUpdate, this, ph::_1)));
#endif
  addConnection(Signal<OpenPcdFileAction>::connect(std::bind(&Clouds::onPcdFileOpen, this, ph::_1)));
}


void Clouds::onPointsUpdate(const UpdatePointsAction &action) {
  std::lock_guard<std::mutex> lg(cloud_mutex_);
  if (clouds_.find(action.key) != clouds_.end()) {
    clouds_[action.key]->set_point_cloud(action.point_cloud);
  } else {
    clouds_[action.key] = std::make_shared<Cloud>(action.key, action.point_cloud);
  }
  emit();
}

void Clouds::onVerticesUpdate(const UpdateVerticesAction &action) {
  std::lock_guard<std::mutex> lg(cloud_mutex_);
  if (clouds_.find(action.key) != clouds_.end()) {
    clouds_[action.key]->set_vertices(action.vertices);
  } else {
    clouds_[action.key] = std::make_shared<Cloud>(action.key, action.vertices);
  }
  emit();
}

void Clouds::onCalibrationParamsUpdate(const UpdateCalibrationParamsAction &action) {
  calib_params_map_[action.key] = action.params;
  emit();
}

void Clouds::onCloudVisibilityChange(const ChangeCloudVisibilityAction &action) {
  clouds_[action.key]->set_visible(action.visible);
  emit();
}

void Clouds::onCloudRemove(const RemoveCloudAction &action) {
  clouds_.erase(action.key);
  emit();
}

void Clouds::onCloudsClear(const ClearCloudsAction &action) {
  clouds_.clear();
  emit();
}

void Clouds::onPassThroughFilterParamsUpdate(const UpdatePassThroughFilterParamsAction &action) {
  if (action.field == "x") {
    x_pass_through_filter_.setParams(action.params);
  } else if (action.field == "y") {
    y_pass_through_filter_.setParams(action.params);
  } else if (action.field == "z") {
    z_pass_through_filter_.setParams(action.params);
  }
  emit();
}

#ifdef USE_NITE2
void Clouds::onUsersThroughFitlerParamsUpdate(const UpdateUsersThroughFitlerParamsAction &action) {
  enable_users_through_filter_ = action.enable;
  emit();
}
#endif

void Clouds::onPcdFileOpen(const OpenPcdFileAction &action) {
  Cloud::PointCloudPtr cloud(new Cloud::PointCloud);
  pcl::io::loadPCDFile(action.path, *cloud);
  clouds_[action.path] = std::make_shared<Cloud>(action.path, cloud);
  emit();
}
