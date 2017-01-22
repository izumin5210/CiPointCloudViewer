//
// Created by Masayuki IZUMI on 10/27/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_JOINT_H
#define CIPOINTCLOUDVIEWERAPP_JOINT_H

#include <map>
#include <msgpack.hpp>

#ifdef USE_NITE2
#include <NiTE.h>
#endif

struct Joint {
  float x;
  float y;
  float z;
  int type;
  MSGPACK_DEFINE(x, y, type);
};

struct Bone {
public:
  enum struct Type {
    HEAD,
    TORSO,
    LEFT_UP_ARM,
    RIGHT_UP_ARM,
    LEFT_DOWN_ARM,
    RIGHT_DOWN_ARM,
    LEFT_UP_LEG,
    RIGHT_UP_LEG,
    LEFT_DOWN_LEG,
    RIGHT_DOWN_LEG,
  };

  static const std::multimap<Type, std::pair<int, int>> joint_pairs;
};

using Skeleton = std::map<int, Joint>;
using Skeletons = std::map<int, Skeleton>;
using SkeletonsPtr = std::shared_ptr<Skeletons>;

#endif //CIPOINTCLOUDVIEWERAPP_JOINT_H
