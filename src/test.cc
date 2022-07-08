#include "../include/pointclouds.h"
#include <boost/ut.hpp>
#include <fmt/format.h>
#include <zpp_bits.h>
#include <fstream>
#include <spdlog/spdlog.h>

int main() {
  using namespace boost::ut;
  using namespace boost::ut::literals;

  using namespace bob::types;

  // load the demo.pc file, which is an uncompressed, serialized PointCloud

  // std::ifstream demo_file("demo.pc", std::ios::in | std::ios::binary);
  // demo_file.seekg(0, std::ios::end);
  // auto demo_file_size = demo_file.tellg();
  // demo_file.seekg(0, std::ios::beg);
  // bytes demo_buffer(demo_file_size);
  // demo_file.read((char*) demo_buffer.data(), demo_file_size);
  // spdlog::info("demo.pc buffer size: {}", demo_buffer.size());
  // auto demo_point_cloud = PointCloud::deserialize(demo_buffer);
  // spdlog::info("demo.pc point count: {}", demo_point_cloud.size());
  // return 0;


  "initialisation"_test = [] {
    PointCloud point_cloud {
	{{-1000, 2000, 3000}, {5, 0, -400}, {100, 20, 56}},
	{{255, 200, 10, 255}, {0, 100, 200, 127}, {255, 255, 255, 50}}
    };
    expect(point_cloud.size() == 3);
    expect(!point_cloud.empty());
    expect(point_cloud.positions[1].z == -400);
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

  "serialization"_test = [&point_cloud] (bool with_compression) {
    bytes buffer = point_cloud.serialize(with_compression);
    expect(!buffer.empty());
  } | compression_flags;

  "deserialization"_test = [&point_cloud] (bool with_compression) {
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

  // "demo_point_cloud_in_out"_test = [&demo_point_cloud] (bool with_compression) {
  // } | compression_flags;
}
