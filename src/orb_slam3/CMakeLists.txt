cmake_minimum_required(VERSION 3.8)
project(orb_slam3)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_compile_options(-Wall -O3)
# Suppress noisy warnings from upstream (unmodified) ORB-SLAM3 / g2o source:
#   reorder      - member init order differs from declaration order
#   sign-compare - signed/unsigned comparison (loop counters vs .size())
#   unused-*     - leftover debug timing vars and unused locals
add_compile_options(
  -Wno-deprecated-declarations
  -Wno-reorder
  -Wno-sign-compare
  -Wno-unused-variable
  -Wno-unused-but-set-variable
)

find_package(ament_cmake REQUIRED)
find_package(orb_slam3_dbow2 REQUIRED)
find_package(orb_slam3_g2o REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.1 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)

set(SOURCES
  src/System.cc
  src/Tracking.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  src/Converter.cc
  src/MapPoint.cc
  src/KeyFrame.cc
  src/Atlas.cc
  src/Map.cc
  src/Optimizer.cc
  src/Frame.cc
  src/KeyFrameDatabase.cc
  src/Sim3Solver.cc
  src/ImuTypes.cc
  src/G2oTypes.cc
  src/CameraModels/Pinhole.cpp
  src/CameraModels/KannalaBrandt8.cpp
  src/OptimizableTypes.cpp
  src/MLPnPsolver.cpp
  src/GeometricTools.cc
  src/TwoViewReconstruction.cc
  src/Config.cc
  src/Settings.cc
)

add_library(orb_slam3 SHARED ${SOURCES})

target_include_directories(orb_slam3
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/orb_slam3>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/orb_slam3/CameraModels>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/orb_slam3>
    $<INSTALL_INTERFACE:include/orb_slam3/CameraModels>
)

ament_target_dependencies(orb_slam3 PUBLIC
  orb_slam3_dbow2
  orb_slam3_g2o
  Eigen3
  OpenCV
)

target_link_libraries(orb_slam3 PUBLIC Boost::serialization OpenSSL::Crypto)

# Build-time tool: text vocabulary -> binary vocabulary
add_executable(vocab_text_to_bin tools/vocab_text_to_bin.cc)
target_link_libraries(vocab_text_to_bin PRIVATE orb_slam3)

# Vocabulary: extract at build time, convert to binary, and install both to
# share/orb_slam3/vocabulary/. The binary form loads ~20x faster than text.
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ORBvoc.txt
  COMMAND ${CMAKE_COMMAND} -E tar xzf
          ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/ORBvoc.txt.tar.gz
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/ORBvoc.txt.tar.gz
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Extracting ORB vocabulary"
)
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ORBvoc.bin
  COMMAND $<TARGET_FILE:vocab_text_to_bin>
          ${CMAKE_CURRENT_BINARY_DIR}/ORBvoc.txt
          ${CMAKE_CURRENT_BINARY_DIR}/ORBvoc.bin
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/ORBvoc.txt vocab_text_to_bin
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Converting ORB vocabulary to binary format"
)
add_custom_target(orb_slam3_vocabulary ALL
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/ORBvoc.txt
          ${CMAKE_CURRENT_BINARY_DIR}/ORBvoc.bin
)

install(TARGETS orb_slam3
  EXPORT export_orb_slam3
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/orb_slam3 include/sophus
  DESTINATION include
  FILES_MATCHING
    PATTERN "*.h"
    PATTERN "*.hpp"
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/ORBvoc.txt
  ${CMAKE_CURRENT_BINARY_DIR}/ORBvoc.bin
  DESTINATION share/${PROJECT_NAME}/vocabulary
)

ament_export_targets(export_orb_slam3 HAS_LIBRARY_TARGET)
ament_export_include_directories(include include/orb_slam3 include/orb_slam3/CameraModels)
ament_export_dependencies(orb_slam3_dbow2 orb_slam3_g2o eigen3_cmake_module Eigen3 OpenCV)

ament_package()
