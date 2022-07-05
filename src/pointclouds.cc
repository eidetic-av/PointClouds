#include "../include/pointclouds.h"
#include "draco/attributes/point_attribute.h"
#include <chrono>
#include <zpp_bits.h>
#include <draco/point_cloud/point_cloud_builder.h>
#include <draco/compression/encode.h>
#include <draco/compression/decode.h>

#include <spdlog/spdlog.h>

namespace bob::types {

  using namespace std::chrono;

  auto PointCloud::serialize(bool compress) -> bytes {
    auto now = system_clock::now().time_since_epoch();
    auto point_count = size();
    PointCloudPacket packet {
      duration_cast<milliseconds>(now).count(),
      point_count,
      compress
    };
    if (compress) {
      // PointCloud::compress is implemented by whatever codec we choose to
      // build the lib with
      packet.data = this->compress();
    } else {
      // if we don't want to compress our point-cloud, the internal data
      // buffer will contain first the positions, then the colors
      auto positions_size = sizeof(short3) * point_count;
      auto colors_size = sizeof(color) * point_count;
      packet.data.resize(positions_size + colors_size);
      memcpy(packet.data.data(), positions.data(), positions_size);
      memcpy(packet.data.data() + positions_size, colors.data(), colors_size);
    }
    // now the packet is initialised with either raw or draco data, we serialize it
    // using zpp_bits (because zpp_bits is really fast)
    auto [output_data, zpp_serialize] = zpp::bits::data_out();
    zpp_serialize(packet).or_throw();

    return output_data;
  }

  auto PointCloud::deserialize(const bytes &buffer) -> PointCloud {
    // first, deserialize to a PointCloudPacket
    PointCloudPacket packet;
    auto zpp_deserialize = zpp::bits::in(buffer);
    zpp_deserialize(packet).or_throw();
    auto point_count = packet.point_count;
    // then decode the internal data buffer differently based on whether the
    // packet is compressed or not, and move the resulting points and colors
    // into a PointCloud class instance
    PointCloud point_cloud;
    if (packet.compressed) {
      // PointCloud::decompress is implemented by whatever codec we choose to
      // build the lib with
      point_cloud = PointCloud::decompress(packet.data, point_count);
    } else {
      // it it's not compressed, our encoding is first the point count, then
      // the vectors of positions and colors
      auto positions_size = sizeof(short3) * point_count;
      auto input_positions = std::bit_cast<short3 *>(packet.data.data());
      auto input_colors = std::bit_cast<color *>(packet.data.data() + positions_size);
      point_cloud.positions.assign(input_positions, input_positions + point_count);
      point_cloud.colors.assign(input_colors, input_colors + point_count);
    }
    return point_cloud;
  }
}
