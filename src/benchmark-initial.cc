#include <cstdint>
#include <fstream>
#include <iterator>
#include <vector>
#include <zpp_bits.h>
#include "../bitshuffle/src/bitshuffle.h"
#include "../bitshuffle/src/bitshuffle_core.h"
#include <spdlog/spdlog.h>

struct short3 {
  short x, y, z = 0;
};

struct PointCloud {
  std::vector<short3> positions;
  std::vector<float> colors;
};

int main() {
  std::ifstream input_file("../test_0.pos", std::ios::binary);
  std::vector<unsigned char> buffer(std::istreambuf_iterator<char>(input_file), {});
  std::vector<uint8_t> bytes(buffer.size());

  spdlog::info("file size: {}", buffer.size());

  auto bytes_ptr = static_cast<uint8_t*>(buffer.data());
  bytes.assign(bytes_ptr, bytes_ptr + buffer.size());
  auto in = zpp::bits::in(bytes);

  std::vector<short3> positions;
  auto zpp_result = in(positions);

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
			block_size, 0);

  spdlog::info("Shuffled zstd compressed size: {}", zstd_output_size);

  std::vector<std::byte> shuffled_zstd_positions_two(shuffled_zstd_bound);

  auto zstd_output_ptr_two = reinterpret_cast<void*>(shuffled_zstd_positions_two.data());

  auto zstd_output_size_two =
    bshuf_compress_zstd(input_ptr, zstd_output_ptr_two, positions_count, elem_size,
			block_size, 22);

  spdlog::info("Shuffled zstd compressed size two: {}", zstd_output_size_two);

  bshuf_compress_zstd_free();
}
