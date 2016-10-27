//
// Created by Masayuki IZUMI on 10/7/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_OPENNI2CLOUDDATASOURCE_H
#define CIPOINTCLOUDVIEWERAPP_OPENNI2CLOUDDATASOURCE_H

#include <condition_variable>
#include <mutex>

#include <opencv2/core.hpp>

#include "CloudDataSource.h"
#include "Skeleton.h"

namespace io {
namespace datasource {

class OpenNI2CloudDataSource :
  public CloudDataSource
  , public openni::VideoStream::NewFrameListener
#ifdef USE_NITE2
  , public nite::UserTracker::NewFrameListener
#endif
{
public:
  OpenNI2CloudDataSource(
    const std::string name,
    const std::string uri
  );

  void onNewFrame(openni::VideoStream &stream) override;
#ifdef USE_NITE2
  void onNewFrame(nite::UserTracker &tracker) override;
#endif


protected:
  void onStart() override;
  void onStop() override;
  void update() override;
  std::shared_ptr<openni::VideoStream> getColorVideoStream() override;
  std::shared_ptr<openni::VideoStream> getDepthVideoStream() override;


private:
  const std::string uri_;

  openni::Device device_;
  const std::shared_ptr<openni::VideoStream> color_stream_;
  const std::shared_ptr<openni::VideoStream> depth_stream_;
  const std::shared_ptr<openni::VideoStream> ir_stream_;

  cv::Mat color_image_;
  cv::Mat raw_depth_image_;
  cv::Mat depth_image_;
  cv::Mat ir_image_;

#ifdef USE_NITE2
  nite::UserTracker user_tracker_;
  const nite::UserId *user_ids_;
  SkeletonsPtr skeletons_;
#endif

  std::mutex mutex_color_image_;
  std::mutex mutex_raw_depth_image_;
  std::mutex mutex_depth_image_;
  std::mutex mutex_ir_image_;
  std::mutex mutex_user_image_;
  std::mutex mutex_updated_;
  std::condition_variable updated_cond_;

  bool updated_color_image_;
  bool updated_depth_image_;
  bool updated_ir_image_;
#ifdef USE_NITE2
  bool updated_user_image_;
#endif


  void startColorStream();
  void startDepthStream();
  void startIrStream();
#ifdef USE_NITE2
  void startUserTracker();
#endif
  void enableMirroring();
  void enableDepthToColorRegistration();
  void updateColorImage(const openni::VideoFrameRef &color_frame);
  void updateRawDepthImage(const openni::VideoFrameRef &depth_frame);
  void updateDepthImage(const openni::VideoFrameRef &depth_frame);
  void updateIrImage(const openni::VideoFrameRef &ir_frame);
#ifdef USE_NITE2
  void updateUserImage(const nite::UserTrackerFrameRef &user_frame);
#endif
  void updatePointCloud(std::chrono::system_clock::time_point timestamp);
};

}
}


#endif //CIPOINTCLOUDVIEWERAPP_OPENNI2CLOUDDATASOURCE_H
