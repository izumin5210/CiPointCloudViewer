//
// Created by Masayuki IZUMI on 2016/09/09.
//

#ifndef CIPOINTCLOUDVIEWERAPP_CLOUDDATASOURCESIMPL_H
#define CIPOINTCLOUDVIEWERAPP_CLOUDDATASOURCESIMPL_H

#include "Dispatcher.h"
#include "io/CloudDataSources.h"

fruit::Component<
    fruit::Required<Dispatcher>,
    io::CloudDataSources
>
getCloudDataSourcesImplComponent();

#endif //CIPOINTCLOUDVIEWERAPP_CLOUDDATASOURCESIMPL_H
