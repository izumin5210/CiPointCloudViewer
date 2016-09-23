//
// Created by Masayuki IZUMI on 7/19/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_CLOUDS_H
#define CIPOINTCLOUDVIEWERAPP_CLOUDS_H

#include <map>

#include "Cloud.h"
#include "Store.h"

#include "io/CalibrationParamsManager.h"
#include "filter/PassThroughFilter.hpp"
#include "filter/VoxelFilter.hpp"
#include "filter/StatisticalOutlierRemovalFilter.hpp"

class Clouds : public Store {
public:
  using Key           = Cloud::Key;
  using PointT        = Cloud::PointT;
  using PointCloudPtr = Cloud::PointCloudPtr;

  virtual void lock() = 0;
  virtual void unlock() = 0;
  virtual std::map<Key, CloudPtr> clouds() const = 0;
  virtual std::map<Key, io::CalibrationParams> calib_params_map() const = 0;
  virtual filter::PassThroughFilter<PointT>::Params x_pass_through_filter_params() const = 0;
  virtual filter::PassThroughFilter<PointT>::Params y_pass_through_filter_params() const = 0;
  virtual filter::PassThroughFilter<PointT>::Params z_pass_through_filter_params() const = 0;
  virtual filter::VoxelFilter<PointT>::Params voxel_filter_params() const = 0;
  virtual filter::StatisticalOutlierRemovalFilter<PointT>::Params sor_filter_params() const = 0;
  virtual size_t size() const = 0;
};

#endif //CIPOINTCLOUDVIEWERAPP_CLOUDS_H
