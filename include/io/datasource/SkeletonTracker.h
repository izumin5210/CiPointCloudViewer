//
// Created by Masayuki IZUMI on 10/27/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_SKELETONTRACKER_H
#define CIPOINTCLOUDVIEWERAPP_SKELETONTRACKER_H

#ifdef USE_NITE2

#include <NiTE.h>

#include "Skeleton.h"

namespace io {
namespace datasource {

class SkeletonTracker {
public:
  static SkeletonsPtr getSkeletons(nite::UserTracker &tracker, const nite::UserTrackerFrameRef &user_frame);


private:
  static Joint getJointPosition(nite::UserTracker &tracker, const nite::SkeletonJoint &joint);
};

}
}

#endif

#endif //CIPOINTCLOUDVIEWERAPP_SKELETONTRACKER_H
