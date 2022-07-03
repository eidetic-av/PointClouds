#pragma once

#include <cstddef>
#include <vector>

namespace bob::types {

  struct short3 {
    short x, y, z = 0;
  };

  struct color {
    unsigned char r, g, b, a = 0;
  };

  using bytes = std::vector<std::byte>;

  struct PointCloudPacket {
    long timestamp;
    bool compressed;
    bytes data;
  };

  class PointCloud {
  public:
    std::vector<short3> positions;
    std::vector<color> colors;

    auto size() { return positions.size(); }
    auto empty() { return positions.empty(); }

    auto serialize(bool compress = false) -> bytes;
    static auto deserialize(const bytes &buffer) -> PointCloud;

  private:
    void resize(std::size_t size);
  };

}
