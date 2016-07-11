//
//  CloudsManager.h
//  CiPointCloudViewer
//
//  Created by Masayuki IZUMI on 12/8/16.
//
//

#ifndef CloudsManager_h
#define CloudsManager_h

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "cinder/Signals.h"

namespace models {

typedef pcl::PointXYZRGBA PointT;
typedef typename pcl::PointCloud<PointT> PointCloud;
typedef typename PointCloud::Ptr PointCloudPtr;

struct CloudEvent {
    std::string key;
    PointCloudPtr cloud;
};

class CloudsManager {
public:
    typedef typename ci::signals::Signal<void (const CloudEvent&)> EventSignalCloud;

    static EventSignalCloud& getSignalCloudUpdated() {
        return get()->signal_cloud_updated_;
    }

    static void emitSignalCloudUpdated(const CloudEvent &event) {
        get()->signal_cloud_updated_.emit(event);
    }

    static CloudsManager* get() {
        static CloudsManager *manager = 0;
        if(!manager) {
          manager = new CloudsManager;
        }
        return manager;
    }

private:
    EventSignalCloud signal_cloud_updated_;

    CloudsManager() {}
    ~CloudsManager() {}
};

}

#endif /* CloudsManager_h */
