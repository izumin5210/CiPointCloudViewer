//
//  StatisticalOutlierRemovalFilter.hpp
//  CiPointCloudViewer
//
//  Created by Masayuki IZUMI on 7/7/16.
//
//

#ifndef StatisticalOutlierRemovalFilter_hpp
#define StatisticalOutlierRemovalFilter_hpp

#include <pcl/filters/statistical_outlier_removal.h>

#include "Filter.hpp"

namespace filter {

template<typename PointT>
class StatisticalOutlierRemovalFilter : public Filter<PointT> {
public:
    struct Params {
        bool enable = false;
        int mean_k  = 50;
        float stddev_mul_threshold = 1.0f;
    };

    Params params_;

    StatisticalOutlierRemovalFilter()
        : filter_(new pcl::StatisticalOutlierRemoval<PointT>())
        , params_()
    {
    }


protected:
    inline std::shared_ptr<pcl::Filter<PointT>> buildFilter() override {
        filter_->setMeanK(params_.mean_k);
        filter_->setStddevMulThresh(params_.stddev_mul_threshold);
        return filter_;
    }


private:
    std::shared_ptr<pcl::StatisticalOutlierRemoval<PointT>> filter_;
};

}

#endif /* StatisticalOutlierRemovalFilter_hpp */
