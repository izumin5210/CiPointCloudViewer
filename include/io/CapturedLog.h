//
// Created by Masayuki IZUMI on 10/29/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_CAPTUREDLOG_H
#define CIPOINTCLOUDVIEWERAPP_CAPTUREDLOG_H

#include <map>

#include "Cloud.h"

namespace io {

class CapturedLog {
public:
  struct UpdateTimestampAction {
    int64_t timestamp;
  };

  struct ResetTimestampAction {
  };

  CapturedLog(
    const std::string &serial,
    const int user_id,
    const std::map<int64_t, CloudPtr> &clouds
  );


private:
  const std::string serial_;
  const int user_id_;
  std::map<int64_t, CloudPtr> clouds_;
  std::string key_;
  std::vector<int64_t> stamps_;
  std::vector<int64_t>::iterator stamp_itr_;

  void initialize();
  void onTimestampUpdate(const UpdateTimestampAction &action);
  void onTimestampReset(const ResetTimestampAction &action);
};

}

#endif //CIPOINTCLOUDVIEWERAPP_CAPTUREDLOG_H
