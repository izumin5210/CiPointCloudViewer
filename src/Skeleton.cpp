//
// Created by Masayuki IZUMI on 11/3/16.
//

#include "Skeleton.h"

const std::multimap<Bone::Type, std::pair<int, int>> Bone::joint_pairs = {
  { Bone::Type::HEAD, { nite::JOINT_HEAD, nite::JOINT_NECK } },
  { Bone::Type::TORSO, { nite::JOINT_NECK, nite::JOINT_LEFT_SHOULDER } },
  { Bone::Type::TORSO, { nite::JOINT_NECK, nite::JOINT_RIGHT_SHOULDER } },
  { Bone::Type::TORSO, { nite::JOINT_NECK, nite::JOINT_TORSO } },
  { Bone::Type::TORSO, { nite::JOINT_TORSO, nite::JOINT_LEFT_SHOULDER } },
  { Bone::Type::TORSO, { nite::JOINT_TORSO, nite::JOINT_RIGHT_SHOULDER } },
  { Bone::Type::TORSO, { nite::JOINT_TORSO, nite::JOINT_LEFT_HIP } },
  { Bone::Type::TORSO, { nite::JOINT_TORSO, nite::JOINT_RIGHT_HIP } },
  { Bone::Type::TORSO, { nite::JOINT_LEFT_HIP, nite::JOINT_RIGHT_HIP } },
  { Bone::Type::LEFT_UP_ARM, { nite::JOINT_LEFT_SHOULDER, nite::JOINT_LEFT_ELBOW } },
  { Bone::Type::LEFT_DOWN_ARM, { nite::JOINT_LEFT_ELBOW, nite::JOINT_LEFT_HAND } },
  { Bone::Type::RIGHT_UP_ARM, { nite::JOINT_RIGHT_SHOULDER, nite::JOINT_RIGHT_ELBOW } },
  { Bone::Type::RIGHT_DOWN_ARM, { nite::JOINT_RIGHT_ELBOW, nite::JOINT_RIGHT_HAND } },
  { Bone::Type::LEFT_UP_LEG, { nite::JOINT_LEFT_HIP, nite::JOINT_LEFT_KNEE } },
  { Bone::Type::LEFT_DOWN_LEG, { nite::JOINT_LEFT_KNEE, nite::JOINT_LEFT_FOOT } },
  { Bone::Type::RIGHT_UP_LEG, { nite::JOINT_RIGHT_HIP, nite::JOINT_RIGHT_KNEE } },
  { Bone::Type::RIGHT_DOWN_LEG, { nite::JOINT_RIGHT_KNEE, nite::JOINT_RIGHT_FOOT } },
};
