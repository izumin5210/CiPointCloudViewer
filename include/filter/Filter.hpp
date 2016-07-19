//
//  Filter.hpp
//  CiPointCloudViewer
//
//  Created by Masayuki IZUMI on 7/7/16.
//
//

#ifndef Filter_hpp
#define Filter_hpp

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>

namespace filter {

template<typename PointT>
class Filter {
public:
  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;

  void filter(PointCloudPtr cloud) {
    PointCloudPtr cloud_filtered(new PointCloud);
    auto filter = buildFilter();
    filter->setInputCloud(cloud);
    filter->filter(*cloud_filtered);
    pcl::copyPointCloud(*cloud_filtered, *cloud);
  }

protected:
  virtual std::shared_ptr<pcl::Filter<PointT>> buildFilter() = 0;
};

}

#endif /* Filter_hpp */
