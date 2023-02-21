cmake_minimum_required(VERSION 3.11)

project(hdrfun
    VERSION 0.0.0
    DESCRIPTION "hdr and renderDAG experiments"
    LANGUAGES CXX C
)
set(HDRFUN_SOVERSION 0)
SET(HDRFUN_RELEASE_CANDIDATE 0)

# set the use of C++17 globally as all examples require it
set(CMAKE_CXX_STANDARD 17)

# template madness

if (MSVC)
    add_compile_options(/bigobj)
endif()

option(BUILD_SHARED_LIBS "Build using shared libraries" OFF)

# Find Vulkan and the VSG
if (VULKAN_SDK)
    set(ENV{VULKAN_SDK} ${VULKAN_SDK})
endif()

set(VSG_MIN_VERSION 1.0.0)
find_package(vsg ${VSG_MIN_VERSION} REQUIRED)

find_package(vsgXchange REQUIRED)

find_package(vsgImGui REQUIRED)

# for generated cmake support files
set(FIND_DEPENDENCY ${FIND_DEPENDENCY} "find_dependency(vsg ${VSG_MIN_VERSION} REQUIRED)")

vsg_setup_build_vars()
# This does include(GNUInstallDirs), which sets up all the correct install directories.
vsg_setup_dir_vars()

set(GENERATED_HEADERS_DIR "${PROJECT_BINARY_DIR}/include")

set(HDRFUN_DATA_DIR "${CMAKE_INSTALL_DATADIR}/hdrfun")
set(HDRFUN_FULL_DATA_DIR "${CMAKE_INSTALL_FULL_DATADIR}/hdrfun")

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/src/hdrfun/Config.h.in"
  "${GENERATED_HEADERS_DIR}/hdrfun/Config.h")

vsg_add_target_clang_format(
  FILES
  ${CMAKE_SOURCE_DIR}/include/*/*.h
  ${CMAKE_SOURCE_DIR}/src/*/*.h
  ${CMAKE_SOURCE_DIR}/src/*/*.cpp
  ${CMAKE_SOURCE_DIR}/applications/*/*.h
  ${CMAKE_SOURCE_DIR}/applications/*/*.cpp
  )
      
vsg_add_target_clobber()
vsg_add_target_docs(
    FILES
        ${CMAKE_SOURCE_DIR}/include
)
vsg_add_target_uninstall()

vsg_add_option_maintainer(
    PREFIX ${PROJECT_NAME}
    RCLEVEL ${HDRFUN_RELEASE_CANDIDATE}
)

# Make the headers visible to everything. This is not OSG / VSG style,
# but we prefer that include files live next to their source files.
include_directories(${hdrfun_SOURCE_DIR}/src ${GENERATED_HEADERS_DIR})

# source directory for library and applications

add_subdirectory(src/hdrfun)
add_subdirectory(data)

vsg_add_feature_summary()
