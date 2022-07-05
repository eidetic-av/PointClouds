cmake_minimum_required(VERSION 3.23.0)

option(BUILD_TESTS "Build tests" OFF)
if(BUILD_TESTS)
  list(APPEND VCPKG_MANIFEST_FEATURES "tests")
endif()

option(WITH_DRACO "Build using Draco codec" ON)
if(WITH_DRACO)
  list(APPEND VCPKG_MANIFEST_FEATURES "draco")
endif()

project(bob-pointclouds VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(TARGET_SUPPORTS_SHARED_LIBS ON)

if (WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdeclspec")
endif()

find_path(ZPP_BITS_INCLUDE_DIRS "zpp_bits.h")
set(SOURCE_FILES src/pointclouds.cc)
set(LINK_LIBS "")

if(WITH_DRACO)
  find_package(draco CONFIG REQUIRED)
  set(SOURCE_FILES ${SOURCE_FILES} src/draco_codec.cc)
  set(LINK_LIBS ${LINK_LIBS} draco)
endif()

add_library(pointclouds ${SOURCE_FILES})
target_compile_definitions(pointclouds PUBLIC
  "POINTCLOUDS_DEBUG=$<CONFIG:Debug>")

target_include_directories(pointclouds PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(pointclouds PRIVATE ${ZPP_BITS_INCLUDE_DIRS})
target_link_libraries(pointclouds PRIVATE ${LINK_LIBS})

# set var that allow us to link easier in projects that consume this lib
set(BOB_POINTCLOUDS_LIB "bob-pointclouds ${LINK_LIBS}" CACHE INTERNAL "")

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

if(BUILD_TESTS)
    find_package(ut CONFIG REQUIRED)
    find_package(fmt CONFIG REQUIRED)
    add_executable(test ${SOURCE_FILES} src/test.cc)
    target_include_directories(test PRIVATE ${ZPP_BITS_INCLUDE_DIRS})
    target_link_libraries(test PRIVATE draco boost::ut fmt::fmt)
endif()

# copy compile_commands.json for lsp integration 
# (assume we don't need this on Windows)
if (NOT WIN32)
  add_custom_target(copy-compile-commands ALL
    ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_BINARY_DIR}/compile_commands.json
    ${CMAKE_CURRENT_LIST_DIR})
endif()
