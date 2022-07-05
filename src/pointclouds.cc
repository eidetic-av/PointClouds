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

  void PointCloud::resize(std::size_t size) {
    positions.resize(size);
    colors.resize(size);
  }

  auto PointCloud::serialize(bool compress) -> bytes {
    auto now = system_clock::now().time_since_epoch();
    auto point_count = size();
    PointCloudPacket packet {
      duration_cast<milliseconds>(now).count(),
      point_count,
      compress
    };
    if (compress) {
      // if we want to compress our point-cloud, the internal data
      // buffer will be in the Draco 3D format
      using namespace draco;
      draco::PointCloudBuilder draco_builder;
      draco_builder.Start(size());
      auto pos_attribute_id = draco_builder.AddAttribute(
	  PointAttribute::POSITION, 3, DataType::DT_INT16);
      draco_builder.SetAttributeValuesForAllPoints(
	  pos_attribute_id, positions.data(), sizeof(short3));
      auto col_attribute_id = draco_builder.AddAttribute(
	  PointAttribute::COLOR, 4, DataType::DT_UINT8);
      draco_builder.SetAttributeValuesForAllPoints(
	  col_attribute_id, colors.data(), sizeof(color));
      // Finalize() below takes a bool specifying if we should run a
      // deduplication step. It's generally too slow to use in real-time
      auto draco_point_cloud = draco_builder.Finalize(false);
      // after moving our point cloud into the draco data type, we can now
      // compress it
      draco::Encoder encoder;
      // the following speed option prioritises decoding speed over
      // both compression ratio and encoding speed
      encoder.SetSpeedOptions(-1, 10);
      draco::EncoderBuffer out_buffer;
      encoder.EncodePointCloudToBuffer(*draco_point_cloud, &out_buffer);
      auto buffer_ptr = std::bit_cast<std::byte*>(out_buffer.data());
      packet.data.assign(buffer_ptr, buffer_ptr + out_buffer.size());
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
      // if it's compressed, it will be in draco encoding
      using namespace draco;
      Decoder decoder;
      DecoderBuffer in_buffer;
      auto buffer_ptr = std::bit_cast<const char *>(packet.data.data());
      in_buffer.Init(buffer_ptr, packet.data.size());
      auto draco_point_cloud = decoder.DecodePointCloudFromBuffer(&in_buffer).value();
      auto positions_attr_id = draco_point_cloud->GetNamedAttributeId(PointAttribute::POSITION);
      auto colors_attr_id = draco_point_cloud->GetNamedAttributeId(PointAttribute::COLOR);
      auto draco_positions = draco_point_cloud->GetAttributeByUniqueId(positions_attr_id);
      auto draco_colors = draco_point_cloud->GetAttributeByUniqueId(colors_attr_id);
      // copy point data from the draco type into our PointCloud
      auto input_positions_ptr = std::bit_cast<short3*>(draco_positions->buffer()->data());
      auto input_colors_ptr = std::bit_cast<color*>(draco_colors->buffer()->data());
      point_cloud.positions.assign(input_positions_ptr, input_positions_ptr + point_count);
      point_cloud.colors.assign(input_colors_ptr, input_colors_ptr + point_count);
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
