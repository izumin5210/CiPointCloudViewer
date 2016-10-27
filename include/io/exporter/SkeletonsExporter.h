//
// Created by Masayuki IZUMI on 10/27/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_SKELETONSEXPORTER_H
#define CIPOINTCLOUDVIEWERAPP_SKELETONSEXPORTER_H

#include "Clouds.h"
#include "Skeleton.h"
#include "io/exporter/Exporter.h"

namespace io {
namespace exporter {

class SkeletonsExporter : public Exporter<SkeletonsPtr> {
public:
  SkeletonsExporter();


protected:
  void initialize();
  void save(const Item &item) override;


private:
  void onSkeletonsUpdate(const Clouds::UpdateSkeletonsAction &action);
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_SKELETONSEXPORTER_H
