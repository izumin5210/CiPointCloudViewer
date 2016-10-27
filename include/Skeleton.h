//
// Created by Masayuki IZUMI on 10/27/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_JOINT_H
#define CIPOINTCLOUDVIEWERAPP_JOINT_H

#include <map>

struct Joint {
  float x;
  float y;
  int type;
};

using Skeleton = std::map<int, Joint>;
using Skeletons = std::map<int, Skeleton>;
using SkeletonsPtr = std::shared_ptr<Skeletons>;

#endif //CIPOINTCLOUDVIEWERAPP_JOINT_H
