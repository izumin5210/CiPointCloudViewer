//
// Created by Masayuki IZUMI on 2016/09/09.
//

#ifndef CIPOINTCLOUDVIEWERAPP_VIEWPARAMSIMPL_H
#define CIPOINTCLOUDVIEWERAPP_VIEWPARAMSIMPL_H

#include "fruit/fruit.h"

#include "Dispatcher.h"
#include "ViewParams.h"

fruit::Component<
    fruit::Required<Dispatcher>,
    ViewParams
>
getViewParamsImplComponent();

#endif //CIPOINTCLOUDVIEWERAPP_VIEWPARAMSIMPL_H
