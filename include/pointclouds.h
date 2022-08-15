#pragma once

#include <cstddef>
#include <vector>

namespace bob::types {

  struct short3 {
    short x, y, z = 0;

    bool operator==(const short3 other) const {
      return x == other.x
	&& y == other.y
	&& z == other.z;
    }
    bool operator!=(const short3 other) const {
      return !operator==(other);
    }
  };

  struct color {
    unsigned char r, g, b, a = 0;

    bool operator==(const color other) const {
      return r == other.r
	&& g == other.g
	&& b == other.b
	&& a == other.a;
    }
    bool operator!=(const color other) const {
      return !operator==(other);
    }
  };

  using bytes = std::vector<std::byte>;

  struct PointCloudPacket {
    unsigned long timestamp;
    unsigned long point_count;
    bool compressed;
    bytes data;
  };

  class PointCloud {
  public:
    std::vector<short3> positions;
    std::vector<color> colors;

    auto size() const { return positions.size(); }
    auto empty() const { return positions.empty(); }

    auto serialize(bool compress = false) const -> bytes;
    static auto deserialize(const bytes &buffer) -> PointCloud;

  private:
    auto compress() -> bytes;
    static auto decompress(const bytes& buffer, unsigned long point_count) -> PointCloud;
  };

  PointCloud operator+(PointCloud const& lhs, PointCloud const& rhs);
  PointCloud operator+=(PointCloud& lhs, const PointCloud& rhs);

}
