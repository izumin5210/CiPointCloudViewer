//
//  CloudsManager.h
//  CiPointCloudViewer
//
//  Created by Masayuki IZUMI on 12/8/16.
//
//

#ifndef CloudsManager_h
#define CloudsManager_h

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "cinder/Signals.h"

namespace models {

typedef pcl::PointXYZRGBA PointT;
typedef typename pcl::PointCloud<PointT> PointCloud;
typedef typename PointCloud::Ptr PointCloudPtr;

struct CloudEvent {
  std::string key;
  PointCloudPtr cloud;
};

}

#endif /* CloudsManager_h */
