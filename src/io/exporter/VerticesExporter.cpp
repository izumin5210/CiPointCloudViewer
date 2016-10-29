//
// Created by Masayuki IZUMI on 10/29/16.
//

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include "Signal.h"
#include "io/exporter/VerticesExporter.h"
#include "util/util.h"

namespace io {
namespace exporter {

VerticesExporter::VerticesExporter(const std::shared_ptr<Clouds> &clouds)
  : Exporter("vertices_exporter")
  , clouds_(clouds)
{
}

void VerticesExporter::save(const Clouds::UpdateVerticesAction &item) {
  const size_t n = static_cast<size_t>(item.user_count + 1);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr *clouds = new pcl::PointCloud<pcl::PointXYZRGBA>::Ptr[n];

  auto stamp = util::to_us(item.timestamp);

  for (size_t i = 0; i < n; i++) {
    util::mkdir_p(dir() / item.key / std::to_string(i));
    clouds[i] = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
  }

  auto calib_params = clouds_->calib_params_map()[item.key];

  for (auto v : *item.vertices) {
    pcl::PointXYZRGBA p;
    p.z = v.xyz[2] / 1000;
    p.x = (v.xyz[0] - calib_params.cx) * p.z / calib_params.fx;
    p.y = (v.xyz[1] - calib_params.cy) * p.z / calib_params.fy;
    p.r = v.rgb[0];
    p.g = v.rgb[1];
    p.b = v.rgb[2];
    clouds[v.user_id]->push_back(p);
  }

  for (size_t i = 0; i < n; i++) {
    if (clouds[i]->empty()) {
      pcl::PointXYZRGBA p;
      p.x = p.y = p.z = p.r = p.g = p.b = p.a = 0;
      clouds[i]->push_back(p);
    }
    auto d = dir() / item.key / std::to_string(i);
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::transformPointCloud(*clouds[i], cloud, calib_params.calib_matrix);
    pcl::io::savePCDFileBinary((d / (std::to_string(stamp) + ".pcd")).string(), cloud);
  }
}

}
}