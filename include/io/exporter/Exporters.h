//
// Created by Masayuki IZUMI on 10/29/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_EXPORTERS_H
#define CIPOINTCLOUDVIEWERAPP_EXPORTERS_H

#include "Clouds.h"
#include "io/exporter/VerticesExporter.h"
#ifdef USE_NITE2
#include "io/exporter/SkeletonsExporter.h"
#endif

namespace io {
namespace exporter {

class Exporters : public Exporter {
public:
  Exporters(const std::shared_ptr<Clouds> &clouds);

  void start(const std::string &dir) override;
  void stopSafety() override;
  void stop() override;
  bool hasStopped() const override;

  const std::unique_ptr<VerticesExporter>& vertices_exporter() {
    return vertices_exporter_;
  };

#ifdef USE_NITE2
  const std::unique_ptr<SkeletonsExporter>& skeletons_exporter() {
    return skeletons_exporter_;
  };
#endif

private:
  const std::unique_ptr<VerticesExporter> vertices_exporter_;
#ifdef USE_NITE2
  const std::unique_ptr<SkeletonsExporter> skeletons_exporter_;
#endif
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_EXPORTERS_H
