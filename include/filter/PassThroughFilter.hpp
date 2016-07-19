//
//  PassThroughFilter.hpp
//  CiPointCloudViewer
//
//  Created by Masayuki IZUMI on 7/7/16.
//
//

#ifndef PassThroughFilter_hpp
#define PassThroughFilter_hpp

#include <pcl/filters/passthrough.h>

#include "Filter.hpp"

namespace filter {

template<typename PointT>
class PassThroughFilter : public Filter<PointT> {
public:
  struct Params {
    bool enable = false;
    float min   = -1.5;
    float max   =  1.5;
  };

  PassThroughFilter(const std::string field_name)
    : filter_(new pcl::PassThrough<PointT>())
    , params_()
  {
    filter_->setFilterFieldName(field_name);
  }

  inline Params params() const {
    return params_;
  }

  inline void setParams(Params params) {
    params_ = params;
  }


protected:
  inline std::shared_ptr<pcl::Filter<PointT>> buildFilter() override {
    filter_->setFilterLimits(params_.min, params_.max);
    return filter_;
  }


private:
  std::shared_ptr<pcl::PassThrough<PointT>> filter_;
  Params params_;
};

}

#endif /* PassThroughFilter_hpp */
