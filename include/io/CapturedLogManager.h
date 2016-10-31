//
// Created by Masayuki IZUMI on 10/29/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_CAPTUREDLOGMANAGER_H
#define CIPOINTCLOUDVIEWERAPP_CAPTUREDLOGMANAGER_H

namespace io {

class CapturedLogManager {
public:
  struct OpenLogAction {
    std::string path;
  };

  static void load(const std::string &path);

  CapturedLogManager();

  std::map<std::string, std::shared_ptr<CapturedLogLoader>> loaders() const {
    return loaders_;
  }


private:
  void onLogOpen(const OpenLogAction &path);
};

}

#endif //CIPOINTCLOUDVIEWERAPP_CAPTUREDLOGMANAGER_H
