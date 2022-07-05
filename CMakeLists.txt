cmake_minimum_required(VERSION 3.23.0)
project(bob-pointclouds VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(TARGET_SUPPORTS_SHARED_LIBS ON)

if (WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdeclspec")
endif()

find_path(ZPP_BITS_INCLUDE_DIRS "zpp_bits.h")
set(SOURCE_FILES src/pointclouds.cc)

# TODO define configuration option for draco lib impl
find_package(draco CONFIG REQUIRED)
set(SOURCE_FILES ${SOURCE_FILES} src/draco_codec.cc)

add_library(pointclouds ${SOURCE_FILES})
target_compile_definitions(pointclouds PUBLIC
  "POINTCLOUDS_DEBUG=$<CONFIG:Debug>")

target_include_directories(pointclouds PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(pointclouds PRIVATE ${ZPP_BITS_INCLUDE_DIRS})
target_link_libraries(pointclouds PRIVATE draco)

# set var that allow us to link easier in projects that consume this lib
set(BOB_POINTCLOUDS_LIB "bob-pointclouds draco" CACHE INTERNAL "")

include(GNUInstallDirs)

install(TARGETS pointclouds
    EXPORT bob-pointclouds-config
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

export(TARGETS pointclouds
    NAMESPACE bob::
    FILE "${CMAKE_CURRENT_BINARY_DIR}/bob-pointclouds-config.cmake"
)
install(EXPORT bob-pointclouds-config
    DESTINATION "${CMAKE_INSTALL_DATADIR}/bob-pointclouds/cmake"
    NAMESPACE bob::
)

find_package(ut CONFIG REQUIRED)
find_package(fmt CONFIG REQUIRED)
add_executable(test ${SOURCE_FILES} src/test.cc)
target_include_directories(test PRIVATE ${ZPP_BITS_INCLUDE_DIRS})
target_link_libraries(test PRIVATE draco boost::ut fmt::fmt)

# copy compile_commands.json for lsp integration 
# (assume we don't need this on Windows)
if (NOT WIN32)
  add_custom_target(copy-compile-commands ALL
    ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_BINARY_DIR}/compile_commands.json
    ${CMAKE_CURRENT_LIST_DIR})
endif()
