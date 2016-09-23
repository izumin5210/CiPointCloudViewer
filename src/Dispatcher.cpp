//
// Created by Masayuki IZUMI on 2016/09/02.
//

#ifndef CIPOINTCLOUDVIEWERAPP_DISPATCHER_CPP
#define CIPOINTCLOUDVIEWERAPP_DISPATCHER_CPP

#include "Dispatcher.h"

fruit::Component<Dispatcher> getDispatcherComponent() {
  Dispatcher dispatcher;
  return fruit::createComponent()
      .bindInstance(dispatcher);
}

#endif //CIPOINTCLOUDVIEWERAPP_DISPATCHER_CPP
