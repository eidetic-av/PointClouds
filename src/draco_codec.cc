#include "../include/pointclouds.h"
#include "draco/attributes/point_attribute.h"
#include <draco/point_cloud/point_cloud_builder.h>
#include <draco/compression/encode.h>
#include <draco/compression/decode.h>

namespace bob::types {

  using namespace draco;

  bytes PointCloud::compress() const {
    PointCloudBuilder draco_builder;
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
    Encoder encoder;
    // the following speed option prioritises decoding speed over
    // both compression ratio and encoding speed
    encoder.SetSpeedOptions(-1, 10);
    EncoderBuffer out_buffer;
    encoder.EncodePointCloudToBuffer(*draco_point_cloud, &out_buffer);
    auto buffer_ptr = std::bit_cast<std::byte*>(out_buffer.data());
    bytes output_data;
    output_data.assign(buffer_ptr, buffer_ptr + out_buffer.size());
    return output_data;
  }

  PointCloud PointCloud::decompress(const bytes &buffer, unsigned long point_count) {
    Decoder decoder;
    DecoderBuffer in_buffer;
    auto buffer_ptr = std::bit_cast<const char *>(buffer.data());
    in_buffer.Init(buffer_ptr, buffer.size());
    auto draco_point_cloud = decoder.DecodePointCloudFromBuffer(&in_buffer).value();
    auto positions_attr_id = draco_point_cloud->GetNamedAttributeId(PointAttribute::POSITION);
    auto colors_attr_id = draco_point_cloud->GetNamedAttributeId(PointAttribute::COLOR);
    auto draco_positions = draco_point_cloud->GetAttributeByUniqueId(positions_attr_id);
    auto draco_colors = draco_point_cloud->GetAttributeByUniqueId(colors_attr_id);
    // copy point data from the draco type into our PointCloud
    auto input_positions_ptr = std::bit_cast<short3*>(draco_positions->buffer()->data());
    auto input_colors_ptr = std::bit_cast<color*>(draco_colors->buffer()->data());
    PointCloud point_cloud;
    point_cloud.positions.assign(input_positions_ptr, input_positions_ptr + point_count);
    point_cloud.colors.assign(input_colors_ptr, input_colors_ptr + point_count);
    return point_cloud;
  }
}
