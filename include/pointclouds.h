#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

namespace bob::types {

 struct uint2 {
   unsigned int x, y = 0;

   bool operator==(const uint2 other) const {
     return x == other.x && y == other.y;
   }
   bool operator!=(const uint2 other) const { return !operator==(other); }
 };

struct short3 {
  short x, y, z = 0;

  bool operator==(const short3 other) const {
    return x == other.x && y == other.y && z == other.z;
  }
  bool operator!=(const short3 other) const { return !operator==(other); }
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

  class PointCloud {
  public:
    std::vector<short3> positions;
    std::vector<color> colors;

    auto size() const { return positions.size(); }
    auto empty() const { return positions.empty(); }

    bytes serialize(bool compress = false) const;
    static PointCloud deserialize(const bytes &buffer);

  private:
    bytes compress() const;
    static PointCloud decompress(const bytes& buffer, unsigned long point_count);
  };

  PointCloud operator+(PointCloud const& lhs, PointCloud const& rhs);
  PointCloud operator+=(PointCloud& lhs, const PointCloud& rhs);

  struct PointCloudPacket {
    // out packet needs these explicitly sized types to ensure portability
    // between unix and windows systems
    uint64_t timestamp;
    uint64_t point_count;
    uint8_t compressed;
    bytes data;
  };

}
