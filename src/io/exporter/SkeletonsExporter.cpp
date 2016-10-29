//
// Created by Masayuki IZUMI on 10/27/16.
//

#include <msgpack.hpp>
#include <sstream>
#include <fstream>

#include <pcl/common/transforms.h>

#include "io/exporter/SkeletonsExporter.h"
#include "util/util.h"

namespace io {
namespace exporter {

SkeletonsExporter::SkeletonsExporter(const std::shared_ptr<Clouds> &clouds)
  : Exporter("skeletons_exporter")
  , clouds_(clouds)
{
}

void SkeletonsExporter::save(const Clouds::UpdateSkeletonsAction &item) {
  std::stringstream ss;
  auto d = dir() / item.key / kDirName;
  util::mkdir_p(d);
  ss << util::to_us(item.timestamp) << ".mpac";
  std::ofstream file((d / ss.str()).string());
  msgpack::pack(&file, calibrate(item.key, item.skeletons));
}

Skeletons SkeletonsExporter::calibrate(const std::string key, SkeletonsPtr skeletons) {
  Skeletons skeletons_calibed;
  auto calib_params = clouds_->calib_params_map()[key];
  for (auto p1 : *skeletons) {
    Skeleton skeleton;
    for (auto p2 : p1.second) {
      pcl::PointCloud<pcl::PointXYZ> cloud, cloud_calibed;
      pcl::PointXYZ p;
      p.z = p2.second.z / 1000;
      p.x = (p2.second.x - calib_params.cx) * p.z / calib_params.fx;
      p.y = (p2.second.y - calib_params.cy) * p.z / calib_params.fy;
      cloud.push_back(p);
      pcl::transformPointCloud(cloud, cloud_calibed, calib_params.calib_matrix);
      p = cloud_calibed.points[0];
      skeleton[p2.first] = { p.x, p.y, p.z, p2.first };
    }
    skeletons_calibed[p1.first] = skeleton;
  }
  return skeletons_calibed;
}

}
}
