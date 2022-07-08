#include "../include/pointclouds.h"
#include <memory>
#include <functional>
#include <zstd.h>
#include <zpp_bits.h>
#include <spdlog/spdlog.h>
#include <iostream>

namespace bob::types {

  // create the zstd contexts and make sure they're deleted when out of scope
  static auto zstd_cctx = std::unique_ptr<ZSTD_CCtx, std::function<std::size_t(ZSTD_CCtx*)>>
    (ZSTD_createCCtx(), &ZSTD_freeCCtx);
  static auto zstd_dctx = std::unique_ptr<ZSTD_DCtx, std::function<std::size_t(ZSTD_DCtx*)>>
    (ZSTD_createDCtx(), &ZSTD_freeDCtx);

  auto PointCloud::compress() -> bytes {
    auto [raw_pointcloud, zpp_serialize] = zpp::bits::data_out();
    zpp_serialize(*this).or_throw();
    auto input_size = raw_pointcloud.size();
    auto compress_bound = ZSTD_compressBound(input_size);
    bytes output_data(compress_bound);
    auto compressed_size =
      ZSTD_compressCCtx(zstd_cctx.get(), output_data.data(), compress_bound,
			raw_pointcloud.data(), input_size, ZSTD_fast);
    output_data.resize(compressed_size);
    return output_data;
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
