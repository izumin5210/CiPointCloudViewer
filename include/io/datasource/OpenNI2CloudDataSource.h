//
// Created by Masayuki IZUMI on 10/7/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_OPENNI2CLOUDDATASOURCE_H
#define CIPOINTCLOUDVIEWERAPP_OPENNI2CLOUDDATASOURCE_H

#include <opencv2/core.hpp>

#include "CloudDataSource.h"

namespace io {
namespace datasource {

class OpenNI2CloudDataSource : public CloudDataSource {
public:
  OpenNI2CloudDataSource(const std::string name, const std::shared_ptr<openni::Device> &device);


protected:
  void onStart() override;
  void onStop() override;
  void update() override;
  std::shared_ptr<openni::VideoStream> getColorVideoStream() override;
  std::shared_ptr<openni::VideoStream> getDepthVideoStream() override;


private:
  const std::shared_ptr<openni::Device> device_;
  const std::shared_ptr<openni::VideoStream> color_stream_;
  const std::shared_ptr<openni::VideoStream> depth_stream_;
  const std::shared_ptr<openni::VideoStream> ir_stream_;

  cv::Mat color_image_;
  cv::Mat raw_depth_image_;
  cv::Mat depth_image_;
  cv::Mat ir_image_;

  void startColorStream();
  void startDepthStream();
  void startIrStream();
  void enableMirroring();
  void enableDepthToColorRegistration();
  void updateColorImage(const openni::VideoFrameRef &color_frame);
  void updateRawDepthImage(const openni::VideoFrameRef &depth_frame);
  void updateDepthImage(const openni::VideoFrameRef &depth_frame);
  void updateIrImage(const openni::VideoFrameRef &ir_frame);
  void updatePointCloud(std::chrono::system_clock::time_point timestamp);
};

}
}


#endif //CIPOINTCLOUDVIEWERAPP_OPENNI2CLOUDDATASOURCE_H
