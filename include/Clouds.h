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
#include "FilterParams.h"
#include "Skeleton.h"

#include "io/CalibrationParamsManager.h"

class Clouds {
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

  struct UpdateSkeletonsAction {
    Key key;
    SkeletonsPtr skeletons;
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
    PassThroughFilterParams params;
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

  PassThroughFilterParams x_pass_through_filter_params() const {
    return x_pass_through_filter_params_;
  }

  PassThroughFilterParams y_pass_through_filter_params() const {
    return y_pass_through_filter_params_;
  }

  PassThroughFilterParams z_pass_through_filter_params() const {
    return z_pass_through_filter_params_;
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

  PassThroughFilterParams x_pass_through_filter_params_;
  PassThroughFilterParams y_pass_through_filter_params_;
  PassThroughFilterParams z_pass_through_filter_params_;
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
