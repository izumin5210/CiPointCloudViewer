//
// Created by izumin on 16/07/22.
//

#ifndef CIPOINTCLOUDVIEWERAPP_VERTEX_H
#define CIPOINTCLOUDVIEWERAPP_VERTEX_H

struct Vertex {
  float xyz[3];
  uint8_t rgb[3];
};

using Vertices    = std::vector<Vertex>;
using VerticesPtr = std::shared_ptr<Vertices>;

#endif //CIPOINTCLOUDVIEWERAPP_VERTEX_H
