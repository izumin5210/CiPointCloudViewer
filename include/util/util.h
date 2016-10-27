//
// Created by Masayuki IZUMI on 10/27/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_UTIL_H_H
#define CIPOINTCLOUDVIEWERAPP_UTIL_H_H

#include <string>

#include "OpenNI.h"
#ifdef USE_NITE2
#include "NiTE.h"
#endif

namespace util {

void checkStatus(openni::Status status, const std::string msg);
#ifdef USE_NITE2
void checkStatus(nite::Status status, const std::string msg);
#endif
void checkStatus(bool is_ok, const std::string msg);

}

#endif //CIPOINTCLOUDVIEWERAPP_UTIL_H_H
