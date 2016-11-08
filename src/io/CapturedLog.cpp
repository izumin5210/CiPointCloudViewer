//
// Created by Masayuki IZUMI on 10/30/16.
//

#include "Clouds.h"
#include "Signal.h"
#include "io/CapturedLog.h"

namespace io {

CapturedLog::CapturedLog(
  const std::string &serial,
  const int user_id,
  const std::map<int64_t, CloudPtr> &clouds
)
  : serial_ (serial)
  , user_id_(user_id)
  , clouds_ (clouds)
  , key_    (serial_ + " (" + std::to_string(user_id) + ")")
{
  initialize();
  Signal<UpdateTimestampAction>::connect(std::bind(&CapturedLog::onTimestampUpdate, this, std::placeholders::_1));
  Signal<ResetTimestampAction>::connect(std::bind(&CapturedLog::onTimestampReset, this, std::placeholders::_1));
}

void CapturedLog::initialize() {
  for (auto pair : clouds_) {
    stamps_.emplace_back(pair.first);
  }
  stamp_itr_ = stamps_.begin();
}

void CapturedLog::onTimestampUpdate(const UpdateTimestampAction &action) {
  if (action.timestamp > *stamp_itr_) {
    if (stamp_itr_ != stamps_.end()) {
      Signal<Clouds::UpdateCloudAction>::emit({key_, clouds_.at(*stamp_itr_)});
      stamp_itr_++;
    }
  }
}

void CapturedLog::onTimestampReset(const ResetTimestampAction &action) {
  stamp_itr_ = stamps_.begin();
  Signal<Clouds::RemoveCloudAction>::emit({key_});
}

}
