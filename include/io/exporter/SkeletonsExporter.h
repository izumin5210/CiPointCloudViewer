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
  SkeletonsExporter(const std::shared_ptr<Clouds> &clouds);


protected:
  void initialize();
  void save(const Item &item) override;


private:
  const std::string kDirName = "skeletons";

  const std::shared_ptr<Clouds> clouds_;

  Skeletons calibrate(const std::string key, SkeletonsPtr skeletons);
  void onSkeletonsUpdate(const Clouds::UpdateSkeletonsAction &action);
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_SKELETONSEXPORTER_H
