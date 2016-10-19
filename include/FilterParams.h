//
// Created by Masayuki IZUMI on 10/19/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_FILTERPARAMS_H
#define CIPOINTCLOUDVIEWERAPP_FILTERPARAMS_H

struct PassThroughFilterParams {
  bool enable = false;
  float min   = -1.5;
  float max   =  1.5;
};

#endif //CIPOINTCLOUDVIEWERAPP_FILTERPARAMS_H
