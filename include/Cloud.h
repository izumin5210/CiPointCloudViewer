//
// Created by Masayuki IZUMI on 8/2/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_CLOUD_H
#define CIPOINTCLOUDVIEWERAPP_CLOUD_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Vertex.h"

class Cloud {
public:
  using PointT        = pcl::PointXYZRGBA;
  using PointCloud    = pcl::PointCloud<PointT>;
  using PointCloudPtr = PointCloud::ConstPtr;
  using Key = std::string;

  Cloud(
    Key key,
    const PointCloudPtr &point_cloud,
    bool calibrated = true,
    bool visible = true
  )
    : key_        (key)
    , point_cloud_(point_cloud)
    , vertices_   (new Vertices)
    , calibrated_ (calibrated)
    , visible_    (visible)
  {
  }

  Cloud(
    Key key,
    const VerticesPtr &vertices,
    bool calibrated = false,
    bool visible = true
  )
    : key_        (key)
    , point_cloud_(new PointCloud)
    , vertices_   (vertices)
    , calibrated_ (calibrated)
    , visible_    (visible)
  {
  }

  inline Key key() const {
    return key_;
  }

  inline PointCloudPtr point_cloud() const {
    return point_cloud_;
  }

  inline void set_point_cloud(const PointCloudPtr &point_cloud) {
    point_cloud_ = point_cloud;
  }

  inline VerticesPtr vertices() const {
    return vertices_;
  }

  inline void set_vertices(const VerticesPtr &vertices) {
    vertices_ = vertices;
  }

  inline bool is_calibrated() const {
    return calibrated_;
  }

  inline void set_calibrated(bool calibrated) {
    calibrated_ = calibrated;
  }

  inline bool is_visible() const {
    return visible_;
  }

  inline void set_visible(bool visible) {
    visible_ = visible;
  }

  inline size_t size() {
    return calibrated_ ? point_cloud_->size() : vertices_->size();
  }

  inline bool needs_render() {
    return (point_cloud()->empty() && vertices()->empty()) || is_visible();
  }


private:
  const Key key_;

  PointCloudPtr point_cloud_;
  VerticesPtr vertices_;

  bool calibrated_;
  bool visible_;
};

using CloudPtr = std::shared_ptr<Cloud>;

#endif //CIPOINTCLOUDVIEWERAPP_CLOUD_H
