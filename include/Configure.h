//
// Created by Masayuki IZUMI on 7/18/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_CONFIGURE_H
#define CIPOINTCLOUDVIEWERAPP_CONFIGURE_H

#include "ViewParams.h"

class Configure {
public:
  virtual void initialize() = 0;

  virtual void setSaveOniFilesTo(std::string dir) = 0;
  virtual std::string getSaveOniFilesTo() const = 0;

  virtual void setSavePcdFilesTo(std::string dir) = 0;
  virtual std::string getSavePcdFilesTo() const = 0;

  virtual void setSaveGridType(ViewParams::Grid grid) = 0;
  virtual ViewParams::Grid getGridType() const = 0;
};

#endif //CIPOINTCLOUDVIEWERAPP_CONFIGURE_H
