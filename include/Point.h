//
// Created by izumin on 16/07/22.
//

#ifndef CIPOINTCLOUDVIEWERAPP_POINT_H
#define CIPOINTCLOUDVIEWERAPP_POINT_H

struct Point {
  float xyz[3];
  uint8_t rgb[3];
};

using Points = std::vector<Point>;

#endif //CIPOINTCLOUDVIEWERAPP_POINT_H
