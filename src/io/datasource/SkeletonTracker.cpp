//
// Created by Masayuki IZUMI on 10/27/16.
//

#ifdef USE_NITE2

#include "io/datasource/SkeletonTracker.h"
#include "util/util.h"

namespace io {
namespace datasource {

SkeletonsPtr SkeletonTracker::getSkeletons(nite::UserTracker &tracker, const nite::UserTrackerFrameRef &user_frame) {
  SkeletonsPtr skeletons(new Skeletons);
  const auto &users = user_frame.getUsers();
  for (int i = 0; i < users.getSize(); i++) {
    const auto &user = users[i];
    if (user.isNew()) {
      util::checkStatus(tracker.startSkeletonTracking(user.getId()), "Failed to start skeleton tracking.");
    } else if (!user.isLost()) {
      {
        const auto &skeleton = user.getSkeleton();
        Skeleton s;
        if (skeleton.getState() != nite::SKELETON_TRACKED) { continue; }
        for (int j = 0; j <= 14; j++) {
          const auto &joint = skeleton.getJoint((nite::JointType) j);
          if (joint.getPositionConfidence() >= 0.7f) {
            s[joint.getType()] = getJointPosition(tracker, joint);
          }
        }
        skeletons->insert(std::make_pair((int) user.getId(), s));
      }
    }
  }

  return skeletons;
}

Joint SkeletonTracker::getJointPosition(nite::UserTracker &tracker, const nite::SkeletonJoint &joint) {
  const auto &pos = joint.getPosition();
  float x = 0, y = 0;
  util::checkStatus(tracker.convertJointCoordinatesToDepth(pos.x, pos.y, pos.z, &x, &y),
                    "Failed to convert joint coords to depth.");
  return { x, y, 0, joint.getType() };
}

}
}

#endif
