//
// Created by Masayuki IZUMI on 10/27/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_UTIL_H_H
#define CIPOINTCLOUDVIEWERAPP_UTIL_H_H

#include <chrono>
#include <string>

#include "boost/filesystem.hpp"

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

bool exists(const std::string &path);
bool exists(const boost::filesystem::path &path);
void mkdir_p(const std::string &dir);
void mkdir_p(const boost::filesystem::path &path);

bool hasExt(const std::string &path, const std::string &ext);
bool hasExt(const boost::filesystem::path &path, const std::string &ext);
void eachFiles(const std::string &path, std::function<void(boost::filesystem::directory_entry&)> proc);
std::string basename(const boost::filesystem::path &path);

std::chrono::system_clock::time_point now();
int64_t to_ms(const std::chrono::system_clock::time_point &tp);
int64_t to_us(const std::chrono::system_clock::time_point &tp);
void sleep(unsigned int ms);

}

#endif //CIPOINTCLOUDVIEWERAPP_UTIL_H_H
