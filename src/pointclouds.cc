#include "../include/pointclouds.h"
#include <chrono>
#include <zpp_bits.h>
#ifndef WITH_CODEC
#include <iostream>
#endif

#include <spdlog/spdlog.h>

namespace bob::types {

  using namespace std::chrono;

  auto PointCloud::serialize(bool compress) -> bytes {
    auto now = system_clock::now().time_since_epoch();
    auto point_count = size();

#ifdef WITH_CODEC
    if (compress) {
      // PointCloud::compress is implemented by whatever codec we choose to
      // build the lib with
      PointCloudPacket packet {
	duration_cast<milliseconds>(now).count(),
	point_count,
	compress,
	this->compress()
      };
      auto [output_data, zpp_serialize] = zpp::bits::data_out();
      zpp_serialize(packet).or_throw();
      return output_data;
    }
#endif

    auto [point_cloud_bytes, serialize_inner] = zpp::bits::data_out();
    serialize_inner(*this).or_throw();
    PointCloudPacket packet {
      duration_cast<milliseconds>(now).count(),
      point_count,
      compress,
      std::move(point_cloud_bytes)
    };
    auto [output_data, zpp_serialize] = zpp::bits::data_out();
    zpp_serialize(packet).or_throw();
    return output_data;
  }

  auto PointCloud::deserialize(const bytes &buffer) -> PointCloud {
    // first, deserialize to a PointCloudPacket
    PointCloudPacket packet;
    auto zpp_deserialize = zpp::bits::in(buffer);
    zpp_deserialize(packet).or_throw();
    // then decode the internal data buffer differently based on whether the
    // packet is compressed or not, and move the resulting points and colors
    // into a PointCloud class instance
#ifdef WITH_CODEC
    if (packet.compressed) {
      // PointCloud::decompress is implemented by whatever codec we choose to
      // build the lib with
      auto point_count = packet.point_count;
      return PointCloud::decompress(packet.data, point_count);
    }
#else
    if (packet.compressed) 
      std::cerr << "You are trying to deserialize a compressed \
	PointCloudPacket, but this library was built without codec support";
#endif
    // if it's not compressed, our packet contains a serialized PointCloud
    PointCloud point_cloud;
    auto deserialize_inner = zpp::bits::in(packet.data);
    deserialize_inner(point_cloud).or_throw();
    return point_cloud;
  }
}
