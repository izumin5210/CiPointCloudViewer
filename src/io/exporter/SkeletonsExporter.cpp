//
// Created by Masayuki IZUMI on 10/27/16.
//

#include <msgpack.hpp>
#include <sstream>
#include <fstream>

#include "io/exporter/SkeletonsExporter.h"

namespace io {
namespace exporter {

SkeletonsExporter::SkeletonsExporter()
  : Exporter("skeletons_exporter")
{
  initialize();
}

void SkeletonsExporter::initialize() {
  Signal<Clouds::UpdateSkeletonsAction>::connect(
    std::bind(&SkeletonsExporter::onSkeletonsUpdate, this, std::placeholders::_1));
}

void SkeletonsExporter::save(const Item &item) {
  std::stringstream ss;
  ss << std::chrono::duration_cast<std::chrono::microseconds>(item.timestamp.time_since_epoch()).count() << ".mpac";
  std::ofstream file((dir() / item.key / ss.str()).string());
  msgpack::pack(&file, *item.item);
}

void SkeletonsExporter::onSkeletonsUpdate(const Clouds::UpdateSkeletonsAction &action) {
  add({action.key, action.timestamp, action.skeletons});
}

}
}
