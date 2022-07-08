#include <cstdint>
#include <draco/attributes/geometry_attribute.h>
#include <draco/core/decoder_buffer.h>
#include <draco/core/draco_types.h>
#include <draco/core/encoder_buffer.h>
#include <fstream>
#include <iterator>
#include <vector>
#include <zpp_bits.h>
#include "../bitshuffle/src/bitshuffle.h"
#include "../bitshuffle/src/bitshuffle_core.h"
#include <spdlog/spdlog.h>
#include <draco/point_cloud/point_cloud_builder.h>
#include <draco/compression/encode.h>
#include <draco/compression/decode.h>
#include <pointcodec.h>

void readPointCloudFromZpp(const char *filename, std::vector<short3> *positions) {
  std::ifstream input_file(filename, std::ios::binary);
  std::vector<unsigned char> buffer(std::istreambuf_iterator<char>(input_file),
				    {});
  std::vector<uint8_t> bytes(buffer.size());
  spdlog::info("file size: {}", buffer.size());
  auto bytes_ptr = static_cast<uint8_t *>(buffer.data());
  bytes.assign(bytes_ptr, bytes_ptr + buffer.size());
  auto in = zpp::bits::in(bytes);
  auto zpp_result = in(*positions);
}

int main() {
  // Grab all of our input point clouds files that conform to a zpp_bits
  // serialized version of our PointCloud data type
  std::vector<short3> positions;
  readPointCloudFromZpp("../test_0.pos", &positions);
  std::vector<short3> positions_b;
  readPointCloudFromZpp("../test_1.pos", &positions_b);
  std::vector<short3> positions_c;
  readPointCloudFromZpp("../test_2.pos", &positions_c);

  std::vector<std::vector<short3>> position_list {
    positions, positions_b, positions_c
  };

  spdlog::info("point count: {}", positions.size());
  spdlog::info("initial positions size: {}", positions.size() * sizeof(short3));

  constexpr auto block_size = 1024;
  const auto positions_count = positions.size() * 3;
  const auto elem_size = sizeof(short);

  auto shuffled_lz4_bound = bshuf_compress_lz4_bound(positions_count, elem_size, block_size);
  spdlog::info("Shuffled LZ4 bound: {}", shuffled_lz4_bound);

  std::vector<std::byte> shuffled_lz4_positions(shuffled_lz4_bound);

  auto input_ptr = reinterpret_cast<const void*>(positions.data());
  auto lz4_output_ptr = reinterpret_cast<void*>(shuffled_lz4_positions.data());

  auto lz4_output_size =
    bshuf_compress_lz4(input_ptr, lz4_output_ptr, positions_count, elem_size, block_size);
  spdlog::info("Shuffled LZ4 compressed size: {}", lz4_output_size);

  auto shuffled_zstd_bound = bshuf_compress_zstd_bound(positions_count, elem_size, block_size);
  spdlog::info("Shuffled zstd bound: {}", shuffled_zstd_bound);

  std::vector<std::byte> shuffled_zstd_positions(shuffled_zstd_bound);

  auto zstd_output_ptr = reinterpret_cast<void*>(shuffled_zstd_positions.data());

  bshuf_compress_zstd_init();

  spdlog::info("init zstd");

  auto zstd_output_size =
    bshuf_compress_zstd(input_ptr, zstd_output_ptr, positions_count, elem_size,
			block_size, 1);

  spdlog::info("Shuffled zstd compressed size: {}", zstd_output_size);

  std::vector<std::byte> shuffled_zstd_positions_two(shuffled_zstd_bound);

  auto zstd_output_ptr_two = reinterpret_cast<void*>(shuffled_zstd_positions_two.data());

  auto zstd_output_size_two =
    bshuf_compress_zstd(input_ptr, zstd_output_ptr_two, positions_count, elem_size,
			block_size, 1);

  spdlog::info("Shuffled zstd compressed size two: {}", zstd_output_size_two);

  bshuf_compress_zstd_free();

  spdlog::info("Moving point cloud into draco::PointCloud");
  auto draco_builder = draco::PointCloudBuilder();
  draco_builder.Start(positions.size());
  draco_builder.AddAttribute(draco::GeometryAttribute::POSITION, 3,
			     draco::DataType::DT_INT16);
  draco_builder.SetAttributeValuesForAllPoints(0, positions.data(), sizeof(short3));
  // passing in "true" runs deduplication on points
  auto draco_point_cloud = draco_builder.Finalize(false);
  spdlog::info("Finished initialising draco::PointCloud");
  draco::Encoder encoder;
  encoder.SetSpeedOptions(-1, 10);
  draco::EncoderBuffer out_buffer;
  spdlog::info("Testing draco encoding with decoding speed set to 10");
  encoder.EncodePointCloudToBuffer(*draco_point_cloud, &out_buffer);
  spdlog::info("Draco pc output size: {}", out_buffer.size());
  spdlog::info("Initialising draco::Decoder");
  draco::Decoder decoder;
  draco::DecoderBuffer in_buffer;
  in_buffer.Init(out_buffer.data(), out_buffer.size());
  spdlog::info("Starting decode");
  auto decode_out = decoder.DecodePointCloudFromBuffer(&in_buffer).value();
  spdlog::info("Finished decode");
  auto output_size = decode_out->attribute(0)->buffer()->data_size();
  spdlog::info("Decoded size (check): {}", output_size);
}
