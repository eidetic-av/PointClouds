#include "../include/pointclouds.h"
#include <boost/ut.hpp>
#include <fmt/format.h>

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
  expect(point_cloud.size() == 2);

  const bool test_compression = false;
  #ifdef WITH_DRACO
  test_compression = true; 
  #endif

  "serialization"_test = [&point_cloud] (bool with_compression) {
    bytes buffer = point_cloud.serialize(with_compression);
    expect(!buffer.empty());
  } | std::vector{ false, test_compression };

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
  } | std::vector{ false, test_compression };
}
