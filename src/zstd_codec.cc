#include "../include/pointclouds.h"
#include <memory>
#include <functional>
#include <zstd.h>
#include <zpp_bits.h>
#include <spdlog/spdlog.h>
#include <iostream>

// #define BITSHUFFLE

#ifdef BITSHUFFLE
#include "../bitshuffle/src/bitshuffle.h"
#include "../bitshuffle/src/bitshuffle_core.h"
#endif

namespace bob::types {

  // create the zstd contexts and make sure they're deleted when out of scope
  static auto zstd_cctx = std::unique_ptr<ZSTD_CCtx, std::function<std::size_t(ZSTD_CCtx*)>>
    (ZSTD_createCCtx(), &ZSTD_freeCCtx);
  static auto zstd_dctx = std::unique_ptr<ZSTD_DCtx, std::function<std::size_t(ZSTD_DCtx*)>>
    (ZSTD_createDCtx(), &ZSTD_freeDCtx);

  auto PointCloud::compress() -> bytes {
#ifdef BITSHUFFLE
    bshuf_compress_zstd_init();
    auto input_ptr = reinterpret_cast<const void*>(positions.data());
    auto positions_count = positions.size();
    auto positions_compress_bound = bshuf_compress_zstd_bound(
	positions_count, sizeof(short3), 0);
    bytes output_data(positions_compress_bound);
    auto output_ptr = reinterpret_cast<void*>(output_data.data());
    auto output_size = bshuf_compress_zstd(input_ptr, output_ptr,
	positions_count, sizeof(short3), 0, 0);
    output_data.resize(output_size);
    spdlog::info("pos.output_size shuffled: {}", output_size);
    // auto colors_count = colors.size();
    // auto colors_size = colors_count * sizeof(color);
    // output_data.resize(output_size + colors_size);
    // auto colors_ptr = reinterpret_cast<const void*>(colors.data());
    // std::memcpy(output_data.data() + output_size, colors_ptr, colors_size);

    auto input_size = positions.size() * sizeof(short3);
    auto compress_bound = ZSTD_compressBound(input_size);
    bytes unshuf_output_data(compress_bound);
    auto compressed_size = ZSTD_compressCCtx(zstd_cctx.get(),
	unshuf_output_data.data(), compress_bound, input_ptr, input_size, ZSTD_fast);
    unshuf_output_data.resize(compressed_size);
    spdlog::info("pos.output_size unshuffled: {}", compressed_size);

    return output_data;
#else
    auto [raw_pointcloud, zpp_serialize] = zpp::bits::data_out();
    zpp_serialize(*this).or_throw();
    auto input_size = raw_pointcloud.size();
    auto compress_bound = ZSTD_compressBound(input_size);
    bytes output_data(compress_bound);
    auto compressed_size = ZSTD_compressCCtx(zstd_cctx.get(),
	output_data.data(), compress_bound, raw_pointcloud.data(), input_size, ZSTD_fast);
    output_data.resize(compressed_size);
    return output_data;
#endif
  }

  auto PointCloud::decompress(const bytes &buffer, unsigned long point_count) -> PointCloud {
    auto compressed_size = buffer.size();
    auto decompressed_size = ZSTD_getDecompressedSize(buffer.data(), compressed_size);
    bytes decompressed(decompressed_size);
    auto output = ZSTD_decompressDCtx(zstd_dctx.get(), decompressed.data(),
				      decompressed_size, buffer.data(), compressed_size);
    if (output == 0) std::cerr << "Failed to decompress zstd encoded point-cloud";
    PointCloud point_cloud;
    auto zpp_deserialize = zpp::bits::in(decompressed);
    zpp_deserialize(point_cloud).or_throw();
    return point_cloud;
  }
}
