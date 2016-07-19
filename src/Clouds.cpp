//
// Created by Masayuki IZUMI on 7/19/16.
//

#include "Clouds.h"
#include "Signal.h"

Clouds::Clouds()
  : cloud_(new PointCloud)
  , x_pass_through_filter_("x")
  , y_pass_through_filter_("y")
  , z_pass_through_filter_("z")
{
  initializeConnections();
}

void Clouds::initializeConnections() {
  addConnection(Signal<UpdateCloudAction>::connect(this, &Clouds::onCloudUpdate));
  addConnection(Signal<ChangeCloudVisibilityAction>::connect(this, &Clouds::onCloudVisibilityChange));
  addConnection(Signal<RemoveCloudAction>::connect(this, &Clouds::onCloudRemove));
  addConnection(Signal<ClearCloudsAction>::connect(this, &Clouds::onCloudsClear));
  addConnection(Signal<UpdatePassThroughFilterParamsAction>::connect(
    this, &Clouds::onPassThroughFilterParamsUpdate));
  addConnection(Signal<UpdateVoxelFilterParamsAction>::connect(
    this, &Clouds::onVoxelFilterParamsUpdate));
  addConnection(Signal<UpdateStatisticalOutlierRemovalFilterParamsAction>::connect(
    this, &Clouds::onStatisticalOutlierRemovalFilterParamsUpdate));
  addConnection(Signal<UpdateCloudLoadingProgressAction>::connect(
    this, &Clouds::onCloudLoadingProgressUpdate));
}

void Clouds::updatePointCloud() {
  std::lock_guard<std::mutex> lg(cloud_mutex_);
  cloud_->clear();

  for (auto pair : clouds_) {
    if (hidden_clouds_.find(pair.first) == hidden_clouds_.end()) {
      *cloud_ += *(pair.second);
    }
  }

  cloud_size_ = cloud_->size();

  if (x_pass_through_filter_params().enable) {
    x_pass_through_filter_.filter(cloud_);
  }

  if (y_pass_through_filter_params().enable) {
    y_pass_through_filter_.filter(cloud_);
  }

  if (z_pass_through_filter_params().enable) {
    z_pass_through_filter_.filter(cloud_);
  }

  if (voxel_filter_params().enable) {
    voxel_filter_.filter(cloud_);
  }

  if (sor_filter_params().enable) {
    sor_filter_.filter(cloud_);
  }

  filtered_cloud_size_ = cloud_->size();
  emit();
}

void Clouds::onCloudUpdate(const UpdateCloudAction &action) {
  clouds_[action.key] = action.cloud;
  updatePointCloud();
}

void Clouds::onCloudVisibilityChange(const ChangeCloudVisibilityAction &action) {
  if (action.visible) {
    hidden_clouds_.erase(action.key);
  } else {
    hidden_clouds_.insert(action.key);
  }
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

void Clouds::onCloudLoadingProgressUpdate(const UpdateCloudLoadingProgressAction &action) {
  loading_progresses_[action.key] = glm::vec2(action.count, action.max);
}
