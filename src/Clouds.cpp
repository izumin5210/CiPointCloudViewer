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
  addConnection(Signal<UpdateVoxelFilterParamsAction>::connect(
    std::bind(&Clouds::onVoxelFilterParamsUpdate, this, ph::_1)));
  addConnection(Signal<UpdateStatisticalOutlierRemovalFilterParamsAction>::connect(
    std::bind(&Clouds::onStatisticalOutlierRemovalFilterParamsUpdate, this, ph::_1)));
  addConnection(Signal<OpenPcdFileAction>::connect(std::bind(&Clouds::onPcdFileOpen, this, ph::_1)));
}

void Clouds::updatePointCloud() {
  std::lock_guard<std::mutex> lg(cloud_mutex_);
//  points_.clear();
//
//  for (auto pair : clouds_) {
//    if (hidden_clouds_.find(pair.first) == hidden_clouds_.end()) {
//      std::copy(pair.second.begin(), pair.second.end(), std::back_inserter(points_));
//    }
//  }
//
//  cloud_size_ = points_.size();
//  filtered_cloud_size_ = points_.size();

//  cloud_->clear();

//  for (auto pair : clouds_) {
//    if (hidden_clouds_.find(pair.first) == hidden_clouds_.end()) {
//      *cloud_ += *(pair.second);
//    }
//  }
//
//  cloud_size_ = cloud_->size();
//
//  if (x_pass_through_filter_params().enable) {
//    x_pass_through_filter_.filter(cloud_);
//  }
//
//  if (y_pass_through_filter_params().enable) {
//    y_pass_through_filter_.filter(cloud_);
//  }
//
//  if (z_pass_through_filter_params().enable) {
//    z_pass_through_filter_.filter(cloud_);
//  }
//
//  if (voxel_filter_params().enable) {
//    voxel_filter_.filter(cloud_);
//  }
//
//  if (sor_filter_params().enable) {
//    sor_filter_.filter(cloud_);
//  }

//  filtered_cloud_size_ = cloud_->size();
  emit();
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
  updatePointCloud();
}

void Clouds::onCloudVisibilityChange(const ChangeCloudVisibilityAction &action) {
  clouds_[action.key]->set_visible(action.visible);
  updatePointCloud();
}

void Clouds::onCloudRemove(const RemoveCloudAction &action) {
  clouds_.erase(action.key);
  updatePointCloud();
}

void Clouds::onCloudsClear(const ClearCloudsAction &action) {
  clouds_.clear();
  updatePointCloud();
}

void Clouds::onPassThroughFilterParamsUpdate(const UpdatePassThroughFilterParamsAction &action) {
  if (action.field == "x") {
    x_pass_through_filter_.setParams(action.params);
  } else if (action.field == "y") {
    y_pass_through_filter_.setParams(action.params);
  } else if (action.field == "z") {
    z_pass_through_filter_.setParams(action.params);
  }
  updatePointCloud();
}

void Clouds::onVoxelFilterParamsUpdate(const UpdateVoxelFilterParamsAction &action) {
  voxel_filter_.setParams(action.params);
  updatePointCloud();
}

void Clouds::onStatisticalOutlierRemovalFilterParamsUpdate(
  const UpdateStatisticalOutlierRemovalFilterParamsAction &action) {
  sor_filter_.setParams(action.params);
  updatePointCloud();
}

void Clouds::onPcdFileOpen(const OpenPcdFileAction &action) {
  Cloud::PointCloudPtr cloud(new Cloud::PointCloud);
  pcl::io::loadPCDFile(action.path, *cloud);
  clouds_[action.path] = std::make_shared<Cloud>(action.path, cloud);
  emit();
}
