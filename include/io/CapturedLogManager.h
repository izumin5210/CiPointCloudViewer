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

private:
  void onLogOpen(const OpenLogAction &path);
};

}

#endif //CIPOINTCLOUDVIEWERAPP_CAPTUREDLOGMANAGER_H
