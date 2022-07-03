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

# TODO static or shared builds switch
# add_library(pointclouds SHARED ${SOURCE_FILES})
add_library(bob-pointclouds STATIC ${SOURCE_FILES})
add_library(bob::pointclouds ALIAS bob-pointclouds)

target_include_directories(bob-pointclouds PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(bob-pointclouds PRIVATE
  ${ZPP_BITS_INCLUDE_DIRS})
target_link_libraries(bob-pointclouds PRIVATE draco)

install(TARGETS bob-pointclouds
  DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})

# copy compile_commands.json for lsp integration 
# (assume we don't need this on Windows)
if (NOT WIN32)
  add_custom_target(copy-compile-commands ALL
    ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_BINARY_DIR}/compile_commands.json
    ${CMAKE_CURRENT_LIST_DIR})
endif()
