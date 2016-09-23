//
// Created by Masayuki IZUMI on 2016/09/08.
//

#ifndef CIPOINTCLOUDVIEWERAPP_CLOUDSACTION_H
#define CIPOINTCLOUDVIEWERAPP_CLOUDSACTION_H

#include "Clouds.h"

struct UpdatePointsAction {
  Clouds::Key key;
  Clouds::PointCloudPtr point_cloud;
};

struct UpdateVerticesAction {
  Clouds::Key key;
  Vertices vertices;
  std::chrono::system_clock::time_point timestamp;
};

struct UpdateCalibrationParamsAction {
  Clouds::Key key;
  io::CalibrationParams params;
};

struct ChangeCloudVisibilityAction {
  Clouds::Key key;
  bool visible;
};

struct RemoveCloudAction {
  Clouds::Key key;
};

struct ClearCloudsAction {
};

struct UpdatePassThroughFilterParamsAction {
  std::string field;
  filter::PassThroughFilter<Clouds::PointT>::Params params;
};

struct UpdateVoxelFilterParamsAction {
  filter::VoxelFilter<Clouds::PointT>::Params params;
};

struct UpdateStatisticalOutlierRemovalFilterParamsAction {
  filter::StatisticalOutlierRemovalFilter<Clouds::PointT>::Params params;
};

struct OpenPcdFileAction {
  std::string path;
};

#endif //CIPOINTCLOUDVIEWERAPP_CLOUDSACTION_H
