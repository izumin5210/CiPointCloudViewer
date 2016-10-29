//
// Created by Masayuki IZUMI on 10/27/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_SKELETONSEXPORTER_H
#define CIPOINTCLOUDVIEWERAPP_SKELETONSEXPORTER_H

#include "Clouds.h"
#include "Skeleton.h"
#include "io/exporter/ExporterBase.h"

namespace io {
namespace exporter {

class SkeletonsExporter : public ExporterBase<Clouds::UpdateSkeletonsAction> {
public:
  SkeletonsExporter(const std::shared_ptr<Clouds> &clouds);


protected:
  void save(const Clouds::UpdateSkeletonsAction &item) override;


private:
  const std::string kDirName = "skeletons";
  const std::shared_ptr<Clouds> clouds_;

  Skeletons calibrate(const std::string key, SkeletonsPtr skeletons);
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_SKELETONSEXPORTER_H
