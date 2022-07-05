#include "../include/pointclouds.h"
#include <zstd.h>

namespace bob::types {

  auto PointCloud::compress() -> bytes {
    bytes output_data;
    return output_data;
  }

  auto PointCloud::decompress(const bytes &buffer, unsigned long point_count) -> PointCloud {
    PointCloud point_cloud;
    return point_cloud;
  }
}
