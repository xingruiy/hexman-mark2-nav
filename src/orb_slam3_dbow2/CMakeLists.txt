cmake_minimum_required(VERSION 3.8)
project(orb_slam3_dbow2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_compile_options(-Wall -O3)

find_package(ament_cmake REQUIRED)
find_package(OpenCV 4 REQUIRED)

add_library(DBoW2 SHARED
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp
  DUtils/Random.cpp
  DUtils/Timestamp.cpp
)

target_include_directories(DBoW2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(DBoW2 PUBLIC ${OpenCV_LIBS})

install(TARGETS DBoW2
  EXPORT export_DBoW2
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY DBoW2 DUtils
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

ament_export_targets(export_DBoW2 HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(OpenCV)

ament_package()
