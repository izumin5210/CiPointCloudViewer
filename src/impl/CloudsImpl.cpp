//
// Created by Masayuki IZUMI on 7/19/16.
//

#include <pcl/io/pcd_io.h>

#include "glm/glm.hpp"

#include "impl/CloudsImpl.h"
#include "action/CloudsAction.h"

class CloudsImpl : public Clouds {
public:
  INJECT(CloudsImpl(std::shared_ptr<Dispatcher> dispatcher))
    : dispatcher_(dispatcher)
    , x_pass_through_filter_("x")
    , y_pass_through_filter_("y")
    , z_pass_through_filter_("z")
  {
    initializeConnections();
  }

  void lock() override {
    cloud_mutex_.lock();
  }

  void unlock() override {
    cloud_mutex_.unlock();
  }

  std::map<Key, CloudPtr> clouds() const override {
    return clouds_;
  };

  std::map<Key, io::CalibrationParams> calib_params_map() const override {
    return calib_params_map_;
  };

  filter::PassThroughFilter<PointT>::Params x_pass_through_filter_params() const override {
    return x_pass_through_filter_.params();
  }

  filter::PassThroughFilter<PointT>::Params y_pass_through_filter_params() const override {
    return y_pass_through_filter_.params();
  }

  filter::PassThroughFilter<PointT>::Params z_pass_through_filter_params() const override {
    return z_pass_through_filter_.params();
  }

  filter::VoxelFilter<PointT>::Params voxel_filter_params() const override {
    return voxel_filter_.params();
  }

  filter::StatisticalOutlierRemovalFilter<PointT>::Params sor_filter_params() const override {
    return sor_filter_.params();
  }

  size_t size() const override {
    size_t size = 0;
    for (auto pair : clouds_) {
      size += pair.second->size();
    }
    return size;
  }

private:
  std::shared_ptr<Dispatcher> dispatcher_;
  std::map<Key, CloudPtr> clouds_;
  std::map<Key, io::CalibrationParams> calib_params_map_;

  filter::PassThroughFilter<PointT> x_pass_through_filter_;
  filter::PassThroughFilter<PointT> y_pass_through_filter_;
  filter::PassThroughFilter<PointT> z_pass_through_filter_;
  filter::VoxelFilter<PointT> voxel_filter_;
  filter::StatisticalOutlierRemovalFilter<PointT> sor_filter_;

  std::map<Key, glm::vec2> loading_progresses_;

  std::mutex cloud_mutex_;

  void initializeConnections() {
    namespace ph = std::placeholders;
    addConnection(dispatcher_->connect<UpdatePointsAction>(std::bind(&CloudsImpl::onPointsUpdate, this, ph::_1)));
    addConnection(dispatcher_->connect<UpdateVerticesAction>(std::bind(&CloudsImpl::onVerticesUpdate, this, ph::_1)));
    addConnection(dispatcher_->connect<UpdateCalibrationParamsAction>(std::bind(&CloudsImpl::onCalibrationParamsUpdate, this, ph::_1)));
    addConnection(dispatcher_->connect<ChangeCloudVisibilityAction>(std::bind(&CloudsImpl::onCloudVisibilityChange, this, ph::_1)));
    addConnection(dispatcher_->connect<RemoveCloudAction>(std::bind(&CloudsImpl::onCloudRemove, this, ph::_1)));
    addConnection(dispatcher_->connect<ClearCloudsAction>(std::bind(&CloudsImpl::onCloudsClear, this, ph::_1)));
    addConnection(dispatcher_->connect<UpdatePassThroughFilterParamsAction>(
        std::bind(&CloudsImpl::onPassThroughFilterParamsUpdate, this, ph::_1)));
    addConnection(dispatcher_->connect<UpdateVoxelFilterParamsAction>(
        std::bind(&CloudsImpl::onVoxelFilterParamsUpdate, this, ph::_1)));
    addConnection(dispatcher_->connect<UpdateStatisticalOutlierRemovalFilterParamsAction>(
        std::bind(&CloudsImpl::onStatisticalOutlierRemovalFilterParamsUpdate, this, ph::_1)));
    addConnection(dispatcher_->connect<OpenPcdFileAction>(std::bind(&CloudsImpl::onPcdFileOpen, this, ph::_1)));
  }

  void addConnection(Dispatcher::Connection connection) {
    // TODO: not yet implemented
  }

  void updatePointCloud() {
    std::lock_guard<std::mutex> lg(cloud_mutex_);
    emit();
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
  }

  void onPointsUpdate(const UpdatePointsAction &action) {
    std::lock_guard<std::mutex> lg(cloud_mutex_);
    if (clouds_.find(action.key) != clouds_.end()) {
      clouds_[action.key]->set_point_cloud(action.point_cloud);
    } else {
      clouds_[action.key] = std::make_shared<Cloud>(action.key, action.point_cloud);
    }
    emit();
  }

  void onVerticesUpdate(const UpdateVerticesAction &action) {
    std::lock_guard<std::mutex> lg(cloud_mutex_);
    if (clouds_.find(action.key) != clouds_.end()) {
      clouds_[action.key]->set_vertices(action.vertices);
    } else {
      clouds_[action.key] = std::make_shared<Cloud>(action.key, action.vertices);
    }
    emit();
  }

  void onCalibrationParamsUpdate(const UpdateCalibrationParamsAction &action) {
    calib_params_map_[action.key] = action.params;
    updatePointCloud();
  }

  void onCloudVisibilityChange(const ChangeCloudVisibilityAction &action) {
    clouds_[action.key]->set_visible(action.visible);
    updatePointCloud();
  }

  void onCloudRemove(const RemoveCloudAction &action) {
    clouds_.erase(action.key);
    updatePointCloud();
  }

  void onCloudsClear(const ClearCloudsAction &action) {
    clouds_.clear();
    updatePointCloud();
  }

  void onPassThroughFilterParamsUpdate(const UpdatePassThroughFilterParamsAction &action) {
    if (action.field == "x") {
      x_pass_through_filter_.setParams(action.params);
    } else if (action.field == "y") {
      y_pass_through_filter_.setParams(action.params);
    } else if (action.field == "z") {
      z_pass_through_filter_.setParams(action.params);
    }
    updatePointCloud();
  }

  void onVoxelFilterParamsUpdate(const UpdateVoxelFilterParamsAction &action) {
    voxel_filter_.setParams(action.params);
    updatePointCloud();
  }

  void onStatisticalOutlierRemovalFilterParamsUpdate(
      const UpdateStatisticalOutlierRemovalFilterParamsAction &action) {
    sor_filter_.setParams(action.params);
    updatePointCloud();
  }

  void onPcdFileOpen(const OpenPcdFileAction &action) {
    Cloud::PointCloudPtr cloud(new Cloud::PointCloud);
    pcl::io::loadPCDFile(action.path, *cloud);
    clouds_[action.path] = std::make_shared<Cloud>(action.path, cloud);
    emit();
  }
};

fruit::Component<
    fruit::Required<Dispatcher>,
    Clouds
>
getCloudsImplComponent() {
  return fruit::createComponent().bind<Clouds, CloudsImpl>();
}

