//
// Created by Masayuki IZUMI on 2016/09/09.
//

#ifndef CIPOINTCLOUDVIEWERAPP_APPGUIIMPL_H
#define CIPOINTCLOUDVIEWERAPP_APPGUIIMPL_H

#include "fruit/fruit.h"

#include "AppGui.h"

#include "Clouds.h"
#include "ViewParams.h"
#include "Configure.h"
#include "SavingVerticesWorker.h"
#include "io/CloudDataSources.h"
#include "io/SensorDeviceManager.h"

fruit::Component<
    fruit::Required<
        Dispatcher,
        Clouds,
        ViewParams,
        Configure,
        io::SensorDeviceManager,
        io::CloudDataSources,
        SavingVerticesWorker
    >,
    AppGui
>
getAppGuiImplComponent();


#endif //CIPOINTCLOUDVIEWERAPP_APPGUIIMPL_H
