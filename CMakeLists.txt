cmake_minimum_required(VERSION 3.23.0)
project(bob-pointclouds)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(TARGET_SUPPORTS_SHARED_LIBS ON)

if (WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdeclspec")
endif()

find_path(ZPP_BITS_INCLUDE_DIRS "zpp_bits.h")

set(SOURCE_FILES src/pointclouds.cc)

# add_library(pointclouds SHARED ${SOURCE_FILES})
add_library(bobpointclouds STATIC ${SOURCE_FILES})

target_include_directories(bobpointclouds PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(bobpointclouds PRIVATE
  ${ZPP_BITS_INCLUDE_DIRS})
target_link_libraries(bobpointclouds PRIVATE draco)

# copy compile_commands.json for lsp integration 
# (assume we don't need this on Windows)
if (NOT WIN32)
  add_custom_target(copy-compile-commands ALL
    ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_BINARY_DIR}/compile_commands.json
    ${CMAKE_CURRENT_LIST_DIR})
endif()
