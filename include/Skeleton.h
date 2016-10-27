//
// Created by Masayuki IZUMI on 10/27/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_JOINT_H
#define CIPOINTCLOUDVIEWERAPP_JOINT_H

#include <map>
#include <msgpack.hpp>

struct Joint {
  float x;
  float y;
  float z;
  int type;
  MSGPACK_DEFINE(x, y, type);
};

using Skeleton = std::map<int, Joint>;
using Skeletons = std::map<int, Skeleton>;
using SkeletonsPtr = std::shared_ptr<Skeletons>;

#endif //CIPOINTCLOUDVIEWERAPP_JOINT_H
