//
// Created by Masayuki IZUMI on 2016/09/09.
//

#ifndef CIPOINTCLOUDVIEWERAPP_CONFIGUREIMPL_H
#define CIPOINTCLOUDVIEWERAPP_CONFIGUREIMPL_H

#include "cinder/app/AppBase.h"
#include "fruit/fruit.h"

#include "Configure.h"
#include "Dispatcher.h"

fruit::Component<
    fruit::Required<Dispatcher, cinder::app::AppBase>,
    Configure
>
getYamlConfigureComponent();

#endif //CIPOINTCLOUDVIEWERAPP_CONFIGUREIMPL_H
