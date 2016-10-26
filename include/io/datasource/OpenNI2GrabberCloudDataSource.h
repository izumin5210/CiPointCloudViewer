//
// Created by Masayuki IZUMI on 10/26/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_OPENNI2GRABBERCLOUDDATASOURCE_H
#define CIPOINTCLOUDVIEWERAPP_OPENNI2GRABBERCLOUDDATASOURCE_H

#include <atomic>
#include <mutex>
#include <condition_variable>

#include <pcl/point_cloud.h>
#include <pcl/io/openni2_grabber.h>

#include "CloudDataSource.h"

namespace io {
namespace datasource {

class OpenNI2GrabberCloudDataSource : public CloudDataSource {
public:
  OpenNI2GrabberCloudDataSource(
    const std::string name,
    const std::string uri
  );


protected:
  void onStart() override;
  void onStop() override;
  void update() override;
  std::shared_ptr<openni::VideoStream> getColorVideoStream() override;
  std::shared_ptr<openni::VideoStream> getDepthVideoStream() override;


private:
  std::unique_ptr<pcl::io::OpenNI2Grabber> grabber_;
  const std::string uri_;
  boost::signals2::connection conn_;

  std::mutex mutex_;
  bool updated_;
  std::condition_variable cond_updated_;

  std::string getDeviceIdForGrabber();
  void onPointCloudReceived(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_OPENNI2GRABBERCLOUDDATASOURCE_H
