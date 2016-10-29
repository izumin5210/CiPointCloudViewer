//
// Created by Masayuki IZUMI on 10/29/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_VERTICESEXPORTER_H
#define CIPOINTCLOUDVIEWERAPP_VERTICESEXPORTER_H

#include "Clouds.h"
#include "Vertex.h"
#include "io/exporter/Exporter.h"

namespace io {
namespace exporter {

class VerticesExporter : public Exporter<Clouds::UpdateVerticesAction> {
public:
  VerticesExporter(const std::shared_ptr<Clouds> &clouds);


protected:
  void save(const Clouds::UpdateVerticesAction &item) override;


private:
  const std::shared_ptr<Clouds> clouds_;
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_VERTICESEXPORTER_H
