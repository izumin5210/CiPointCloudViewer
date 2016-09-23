//
// Created by Masayuki IZUMI on 2016/09/09.
//

#ifndef CIPOINTCLOUDVIEWERAPP_CLOUDSIMPL_H
#define CIPOINTCLOUDVIEWERAPP_CLOUDSIMPL_H

#include "Clouds.h"
#include "Dispatcher.h"

fruit::Component<
  fruit::Required<Dispatcher>,
  Clouds
>
getCloudsImplComponent();

#endif //CIPOINTCLOUDVIEWERAPP_CLOUDSIMPL_H
