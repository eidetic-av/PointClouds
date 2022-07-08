cmake_minimum_required(VERSION 3.23.0)

# --------------------------------------
# parse and set up configuration options
# --------------------------------------

option(WITH_DRACO "Build using Draco codec" ON)
option(WITH_ZSTD "Build using zstd codec" OFF)
if (WITH_ZSTD)
  set(WITH_DRACO OFF)
endif()

set(CODEC_COUNT 0)
if(WITH_DRACO)
  list(APPEND VCPKG_MANIFEST_FEATURES draco)
  list(APPEND SOURCE_FILES src/draco_codec.cc)
  list(APPEND LINK_LIBS draco)
  list(APPEND PACKAGES draco)
  MATH(EXPR CODEC_COUNT "${CODEC_COUNT}+1")
endif()

if(WITH_ZSTD)
  list(APPEND VCPKG_MANIFEST_FEATURES "zstd")
  list(APPEND PACKAGES zstd)
  list(APPEND LINK_LIBS $<IF:$<TARGET_EXISTS:zstd::libzstd_shared>,
    zstd::libzstd_shared,zstd::libzstd_static>)
  list(APPEND SOURCE_FILES src/zstd_codec.cc)
  MATH(EXPR CODEC_COUNT "${CODEC_COUNT}+1")
endif()

if(${CODEC_COUNT} GREATER 1)
  message(FATAL_ERROR
    "CODEC_COUNT > 1: \
    This library can only be configured with a single codec.")
elseif(${CODEC_COUNT} EQUAL 1)
  add_definitions(-DWITH_CODEC)
endif()

option(BUILD_TESTS "Build tests" OFF)
if(BUILD_TESTS)
  list(APPEND VCPKG_MANIFEST_FEATURES "tests")
  list(APPEND TEST_SOURCE src/test.cc)
  list(APPEND TEST_LIBS boost::ut fmt::fmt)
  list(APPEND PACKAGES ut fmt)
endif()

# --------------------------------------
# set up project & compiler settings ---
# --------------------------------------

project(bob-pointclouds VERSION 0.0.1)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(TARGET_SUPPORTS_SHARED_LIBS ON)

if (WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdeclspec")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")

# --------------------------------------
# load vcpkg packages ------------------
# --------------------------------------

foreach(PACKAGE ${PACKAGES})
  find_package(${PACKAGE} CONFIG REQUIRED)
endforeach()

find_path(ZPP_BITS_INCLUDE_DIRS "zpp_bits.h")

# --------------------------------------
# configure library & exports ----------
# --------------------------------------

list(APPEND SOURCE_FILES src/pointclouds.cc)

add_library(pointclouds ${SOURCE_FILES})
target_compile_definitions(pointclouds PUBLIC
  "POINTCLOUDS_DEBUG=$<CONFIG:Debug>")

target_include_directories(pointclouds PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(pointclouds PRIVATE ${ZPP_BITS_INCLUDE_DIRS})
target_link_libraries(pointclouds PRIVATE ${LINK_LIBS})

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
    NAMESPACE bob::
    DESTINATION "${CMAKE_INSTALL_DATADIR}/bob-pointclouds/cmake"
)

# export var that allow us to link easier in projects that consume this lib
set(BOB_POINTCLOUDS_LIB "bob-pointclouds ${LINK_LIBS}" CACHE INTERNAL "")

# --------------------------------------
# configure test executable
# --------------------------------------

if(BUILD_TESTS)
    add_executable(test ${SOURCE_FILES} ${TEST_SOURCE})
    target_include_directories(test PRIVATE ${ZPP_BITS_INCLUDE_DIRS})
    target_link_libraries(test PRIVATE ${LINK_LIBS} ${TEST_LIBS})
    add_custom_command(TARGET test POST_BUILD COMMAND ${CMAKE_COMMAND} -E
      copy_directory ${CMAKE_SOURCE_DIR}/extra/test $<TARGET_FILE_DIR:test>)
endif()

# --------------------------------------
# configure IDE
# --------------------------------------

# copy compile_commands.json for lsp integration 
add_custom_target(copy-compile-commands ALL
  ${CMAKE_COMMAND} -E copy_if_different
  ${CMAKE_BINARY_DIR}/compile_commands.json
  ${CMAKE_CURRENT_LIST_DIR})
