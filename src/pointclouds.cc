#include "../include/pointclouds.h"
#include <chrono>
#include <zpp_bits.h>
#ifndef WITH_CODEC
#include <iostream>
#endif

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

    #ifdef WITH_CODEC
    if (compress) {
      // PointCloud::compress is implemented by whatever codec we choose to
      // build the lib with
      packet.data = this->compress();
      auto [output_data, zpp_serialize] = zpp::bits::data_out();
      zpp_serialize(packet).or_throw();
      return output_data;
    }
    #endif
    // if we don't want to compress our point-cloud, the internal data
    // buffer will contain first the positions, then the colors
    auto positions_size = sizeof(short3) * point_count;
    auto colors_size = sizeof(color) * point_count;
    packet.data.resize(positions_size + colors_size);
    memcpy(packet.data.data(), positions.data(), positions_size);
    memcpy(packet.data.data() + positions_size, colors.data(), colors_size);
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
    #ifdef WITH_CODEC
    if (packet.compressed) {
      // PointCloud::decompress is implemented by whatever codec we choose to
      // build the lib with
      return PointCloud::decompress(packet.data, point_count);
    }
    #else
    if (packet.compressed) 
      std::cerr << "You are trying to deserialize a compressed \
	PointCloudPacket, but this library was built without codec support";
    #endif
    // it it's not compressed, our packet data contains first the positions,
    // then the colors
    auto positions_size = sizeof(short3) * point_count;
    auto input_positions = std::bit_cast<short3 *>(packet.data.data());
    auto input_colors =
	std::bit_cast<color *>(packet.data.data() + positions_size);
    PointCloud point_cloud;
    point_cloud.positions.assign(input_positions,
				 input_positions + point_count);
    point_cloud.colors.assign(input_colors, input_colors + point_count);
    return point_cloud;
  }
}
