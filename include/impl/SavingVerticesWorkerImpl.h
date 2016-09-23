//
// Created by Masayuki IZUMI on 2016/09/09.
//

#ifndef CIPOINTCLOUDVIEWERAPP_SAVINGVERTICESWORKERIMPL_H
#define CIPOINTCLOUDVIEWERAPP_SAVINGVERTICESWORKERIMPL_H

#include "fruit/fruit.h"

#include "SavingVerticesWorker.h"

fruit::Component<
    fruit::Required<Dispatcher>,
    SavingVerticesWorker
>
getSavingVerticesWorkerImplComponent();

#endif //CIPOINTCLOUDVIEWERAPP_SAVINGVERTICESWORKERIMPL_H
