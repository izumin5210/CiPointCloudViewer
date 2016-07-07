//
//  VoxelFilter.hpp
//  CiPointCloudViewer
//
//  Created by Masayuki IZUMI on 7/7/16.
//
//

#ifndef VoxelFilter_hpp
#define VoxelFilter_hpp

#include <pcl/filters/voxel_grid.h>

#include "Filter.hpp"

namespace filter {

template<typename PointT>
class VoxelFilter : public Filter<PointT> {
public:
    struct Params {
        bool enable = false;
        float size  = 0.01f;
    };

    Params params_;

    VoxelFilter()
        : filter_(new pcl::VoxelGrid<PointT>())
        , params_()
    {
    }


protected:
    inline std::shared_ptr<pcl::Filter<PointT>> buildFilter() override {
        filter_->setLeafSize(params_.size, params_.size, params_.size);
        return filter_;
    }


private:
    std::shared_ptr<pcl::VoxelGrid<PointT>> filter_;
};

}

#endif /* VoxelFilter_hpp */
