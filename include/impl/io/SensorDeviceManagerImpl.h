//
// Created by Masayuki IZUMI on 2016/09/09.
//

#ifndef CIPOINTCLOUDVIEWERAPP_SENSORDEVICEMANAGERIMPL_H
#define CIPOINTCLOUDVIEWERAPP_SENSORDEVICEMANAGERIMPL_H

#include "Dispatcher.h"
#include "io/SensorDeviceManager.h"

fruit::Component<
    fruit::Required<Dispatcher>,
    io::SensorDeviceManager
>
getSensorDeviceManagerImplComponent();

#endif //CIPOINTCLOUDVIEWERAPP_SENSORDEVICEMANAGERIMPL_H
