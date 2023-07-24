#include "../include/pointclouds.h"
#include <boost/ut.hpp>
#include <fmt/format.h>
#include <zpp_bits.h>
#include <fstream>

int main() {
  using namespace boost::ut;
  using namespace boost::ut::literals;

  using namespace bob::types;

  "initialisation"_test = [] {
    PointCloud point_cloud {
      {{-1000, 2000, 3000}, {5, 0, -400}, {100, 20, 56}},
	{{255, 200, 10, 255}, {0, 100, 200, 127}, {255, 255, 255, 50}}
    };
    expect(point_cloud.size() == 3);
    expect(!point_cloud.empty());
    expect(point_cloud.positions[1].z == -400);
    expect(point_cloud.positions[3].__pad == 0);
    expect(point_cloud.colors[2].a == 50);
  };

  PointCloud point_cloud;
  expect(point_cloud.empty());
  "assignment"_test = [&point_cloud] {
    point_cloud = {
      {{100, 200, 300}, {923, -200, -10000}},
      {{255, 255, 255, 255}, {200, 184, 222, 180}}
    };
    expect(!point_cloud.empty());
    expect(point_cloud.size() == 2);
  };

  "fill"_test = [&point_cloud] {
    point_cloud.positions.clear();
    point_cloud.colors.clear();
    const int size = 1252019;
    for (int i = 0; i < size; i++) {
      point_cloud.positions.push_back({100, 200, 300});
      point_cloud.colors.push_back({200, 184, 222, 180});
    }
    expect(!point_cloud.empty());
    expect(point_cloud.size() == size);
  };

  std::vector compression_flags { false };
#ifdef WITH_CODEC
  compression_flags.push_back(true);
#endif

  "serialization"_test = [point_cloud] (bool with_compression) {
    bytes buffer = point_cloud.serialize(with_compression);
    expect(!buffer.empty());
  } | compression_flags;

  "deserialization"_test = [point_cloud] (bool with_compression) {
    bytes buffer = point_cloud.serialize(with_compression);
    auto point_cloud_out = PointCloud::deserialize(buffer);
    expect(!point_cloud_out.empty());
    expect(point_cloud_out.size() == point_cloud.size())
      << fmt::format("point_cloud_out.size ({}) != point_cloud.size ({})",
		     point_cloud_out.size(), point_cloud.size());
    int p = 0;
    for (auto out_position : point_cloud_out.positions) {
      auto in_position = point_cloud.positions[p++];
      expect(out_position == in_position) << "lost position value";
    }
    int c = 0;
    for (auto out_color : point_cloud_out.colors) {
      auto in_color = point_cloud.colors[c++];
      expect(out_color == in_color) << "lost color value";
    }
  } | compression_flags;

//   // load the *.pc demo files, which are uncompressed, serialized PointClouds
//   std::vector<PointCloud> demo_clouds;
//   std::vector<unsigned long> demo_cloud_sizes;
//   int demo_count = 50;
//   for (int i = 0; i < demo_count; i++) {
//     auto filename = fmt::format("demo_{}.pc", i);
//     std::ifstream file(filename, std::ios::in | std::ios::binary);
//     file.seekg(0, std::ios::end);
//     auto file_size = file.tellg();
//     file.seekg(0, std::ios::beg);
//     bytes file_buffer(file_size);
//     file.read(std::bit_cast<char*>(file_buffer.data()), file_size);
//     demo_clouds.push_back(PointCloud::deserialize(file_buffer));
//     demo_cloud_sizes.push_back(file_size);
//   }

//   for (int i = 0; i < demo_count; i++) {
//     auto demo_point_cloud = &demo_clouds.at(i);
//     using namespace std::chrono;
//     auto compress_start = high_resolution_clock::now();
//     auto buffer = demo_point_cloud->serialize(true);
//     auto compress_end = high_resolution_clock::now();
//     auto compress_duration = duration_cast<milliseconds>(
// 	compress_end - compress_start);
//     spdlog::info("compressing {} took {}ms", i,
// 	compress_duration.count());
//     auto compression_ratio = static_cast<float>(demo_cloud_sizes.at(i) / buffer.size());
//     spdlog::info(" --> compression ratio: {}", compression_ratio);
//     auto decompress_start = high_resolution_clock::now();
//     auto out_cloud = PointCloud::deserialize(buffer);
//     auto decompress_end = high_resolution_clock::now();
//     auto decompress_duration = duration_cast<milliseconds>(
// 	decompress_end - decompress_start);
//     spdlog::info("decompressing {} took {}ms", i,
// 	decompress_duration.count());
//   }

  // "demo_cloud_in_out"_test = [&demo_clouds] (bool with_compression) {

  // } | compression_flags;
}
