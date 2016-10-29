//
// Created by Masayuki IZUMI on 10/29/16.
//

#include "io/exporter/Exporters.h"
#include "io/exporter/VerticesExporter.h"
#include "io/exporter/SkeletonsExporter.h"

namespace io {
namespace exporter {

Exporters::Exporters(const std::shared_ptr<Clouds> &clouds)
  : vertices_exporter_(new VerticesExporter(clouds))
#ifdef USE_NITE2
  , skeletons_exporter_(new SkeletonsExporter(clouds))
#endif
{
}

void Exporters::start(const std::string &dir) {
  vertices_exporter_->start(dir);
#ifdef USE_NITE2
  skeletons_exporter_->start(dir);
#endif
}

void Exporters::stopSafety() {
  vertices_exporter_->stopSafety();
#ifdef USE_NITE2
  skeletons_exporter_->stopSafety();
#endif
}

void Exporters::stop() {
  vertices_exporter_->stop();
#ifdef USE_NITE2
  skeletons_exporter_->stop();
#endif
}

bool Exporters::hasStopped() const {
#ifdef USE_NITE2
  return vertices_exporter_->hasStopped() && skeletons_exporter_->hasStopped();
#else
  return vertices_exporter_->hasStopped();
#endif
}

}
}
