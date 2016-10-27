//
// Created by Masayuki IZUMI on 10/27/16.
//

#include <stdexcept>
#include "util/util.h"

namespace util {

void checkStatus(openni::Status status, std::string msg) {
  checkStatus(status == openni::STATUS_OK, msg);
}

#ifdef USE_NITE2
void  checkStatus(nite::Status status, std::string msg) {
  checkStatus(status == nite::STATUS_OK, msg);
}
#endif

void checkStatus(bool status_ok, std::string msg) {
  if (!status_ok) {
    throw std::runtime_error(msg);
  }
}

}

