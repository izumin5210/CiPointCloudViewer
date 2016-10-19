//
// Created by Masayuki IZUMI on 7/19/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_CLOUDS_H
#define CIPOINTCLOUDVIEWERAPP_CLOUDS_H

#include <chrono>
#include <map>
#include <mutex>
#include <set>

#include "glm/glm.hpp"

#include "Cloud.h"
#include "Store.h"

#include "io/CalibrationParamsManager.h"

#include "filter/PassThroughFilter.hpp"

class Clouds : public Store {
public:
  using Key           = Cloud::Key;
  using PointT        = Cloud::PointT;
  using PointCloudPtr = Cloud::PointCloudPtr;

  struct UpdatePointsAction {
    Key key;
    PointCloudPtr point_cloud;
  };

  struct UpdateVerticesAction {
    Key key;
    VerticesPtr vertices;
    std::chrono::system_clock::time_point timestamp;
  };

  struct UpdateCalibrationParamsAction {
    Key key;
    io::CalibrationParams params;
  };

  struct ChangeCloudVisibilityAction {
    Key key;
    bool visible;
  };

  struct RemoveCloudAction {
    Key key;
  };

  struct ClearCloudsAction {
  };

  struct UpdatePassThroughFilterParamsAction {
    std::string field;
    filter::PassThroughFilter<PointT>::Params params;
  };

  struct UpdateUsersThroughFitlerParamsAction {
    bool enable;
  };

  struct OpenPcdFileAction {
    std::string path;
  };

  Clouds();

  void lock() {
    cloud_mutex_.lock();
  }

  void unlock() {
    cloud_mutex_.unlock();
  }

  std::map<Key, CloudPtr> clouds() const {
    return clouds_;
  };

  std::map<Key, io::CalibrationParams> calib_params_map() const {
    return calib_params_map_;
  };

  filter::PassThroughFilter<PointT>::Params x_pass_through_filter_params() const {
    return x_pass_through_filter_.params();
  }

  filter::PassThroughFilter<PointT>::Params y_pass_through_filter_params() const {
    return y_pass_through_filter_.params();
  }

  filter::PassThroughFilter<PointT>::Params z_pass_through_filter_params() const {
    return z_pass_through_filter_.params();
  }

  filter::VoxelFilter<PointT>::Params voxel_filter_params() const {
    return voxel_filter_.params();
  }

  filter::StatisticalOutlierRemovalFilter<PointT>::Params sor_filter_params() const {
    return sor_filter_.params();
  }

#ifdef USE_NITE2
  bool enable_users_through_filter() const {
    return enable_users_through_filter_;
  }
#endif

  size_t size() const {
    size_t size = 0;
    for (auto pair : clouds_) {
      size += pair.second->size();
    }
    return size;
  }


private:
  std::map<Key, CloudPtr> clouds_;
  std::map<Key, io::CalibrationParams> calib_params_map_;

  filter::PassThroughFilter<PointT> x_pass_through_filter_;
  filter::PassThroughFilter<PointT> y_pass_through_filter_;
  filter::PassThroughFilter<PointT> z_pass_through_filter_;
#ifdef USE_NITE2
  bool enable_users_through_filter_;
#endif

  std::map<Key, glm::vec2> loading_progresses_;

  std::mutex cloud_mutex_;

  void initializeConnections();

  void onPointsUpdate(const UpdatePointsAction &action);
  void onVerticesUpdate(const UpdateVerticesAction &action);
  void onCalibrationParamsUpdate(const UpdateCalibrationParamsAction &action);
  void onCloudVisibilityChange(const ChangeCloudVisibilityAction &action);
  void onCloudRemove(const RemoveCloudAction &action);
  void onCloudsClear(const ClearCloudsAction &action);
  void onPassThroughFilterParamsUpdate(const UpdatePassThroughFilterParamsAction &action);
#ifdef USE_NITE2
  void onUsersThroughFitlerParamsUpdate(const UpdateUsersThroughFitlerParamsAction &action);
#endif
  void onPcdFileOpen(const OpenPcdFileAction &action);
};

#endif //CIPOINTCLOUDVIEWERAPP_CLOUDS_H
