//
// Created by Masayuki IZUMI on 10/27/16.
//

#include <stdexcept>
#include <boost/range/iterator_range.hpp>
#include "util/util.h"

namespace util {

void checkStatus(openni::Status status, std::string msg) {
  checkStatus(status == openni::STATUS_OK, msg);
}

#ifdef USE_NITE2
void checkStatus(nite::Status status, std::string msg) {
  checkStatus(status == nite::STATUS_OK, msg);
}
#endif

void checkStatus(bool status_ok, std::string msg) {
  if (!status_ok) {
    throw std::runtime_error(msg);
  }
}

bool exists(const std::string &path) {
  return util::exists(boost::filesystem::path(path));
}

bool exists(const boost::filesystem::path &path) {
  return boost::filesystem::exists(path);
}

void mkdir_p(const std::string &dir) {
  mkdir_p(boost::filesystem::path(dir));
}

void mkdir_p(const boost::filesystem::path &path) {
  if (!boost::filesystem::exists(path)) {
    boost::system::error_code error;
    checkStatus(
      boost::filesystem::create_directories(path, error),
      "Failed to create directory."
    );
  }
}

bool hasExt(const std::string &path, const std::string &ext) {
  return hasExt(boost::filesystem::path(path), ext);
}

bool hasExt(const boost::filesystem::path &path, const std::string &ext) {
  return path.extension().string() == ext;
}

void eachFiles(const std::string &path, std::function<void(boost::filesystem::directory_entry&)> proc) {
  auto range = boost::make_iterator_range(
    boost::filesystem::directory_iterator(path),
    boost::filesystem::directory_iterator()
  );
  for (auto file : range) {
    proc(file);
  }
}

std::string basename(const boost::filesystem::path &path) {
  auto name = path.filename().string();
  auto ext = path.extension().string();
  return name.replace(name.find(ext), ext.size(), "");
}

std::chrono::system_clock::time_point now() {
  return std::chrono::system_clock::now();
}

int64_t to_ms(const std::chrono::system_clock::time_point &tp) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch()).count();
}

int64_t to_us(const std::chrono::system_clock::time_point &tp) {
  return std::chrono::duration_cast<std::chrono::microseconds>(tp.time_since_epoch()).count();
}

}
