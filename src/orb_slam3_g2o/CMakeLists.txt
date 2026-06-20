cmake_minimum_required(VERSION 3.8)
project(orb_slam3_g2o)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_compile_options(-Wall -W -O3)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.1 REQUIRED)

# Generate config.h (matches the upstream configure_file step). Upstream headers
# under g2o/core/ include it as "../../config.h", which resolves to <include>/config.h
# at install time — install it at the include root, not under include/g2o/.
set(G2O_OPENMP 0)
set(G2O_SHARED_LIBS 1)
configure_file(config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)

set(G2O_SOURCES
  g2o/types/types_sba.cpp
  g2o/types/types_six_dof_expmap.cpp
  g2o/types/types_seven_dof_expmap.cpp

  g2o/core/hyper_graph_action.cpp
  g2o/core/hyper_graph.cpp
  g2o/core/marginal_covariance_cholesky.cpp
  g2o/core/matrix_structure.cpp
  g2o/core/batch_stats.cpp
  g2o/core/parameter.cpp
  g2o/core/cache.cpp
  g2o/core/optimizable_graph.cpp
  g2o/core/solver.cpp
  g2o/core/optimization_algorithm_factory.cpp
  g2o/core/estimate_propagator.cpp
  g2o/core/factory.cpp
  g2o/core/sparse_optimizer.cpp
  g2o/core/hyper_dijkstra.cpp
  g2o/core/parameter_container.cpp
  g2o/core/optimization_algorithm.cpp
  g2o/core/optimization_algorithm_with_hessian.cpp
  g2o/core/optimization_algorithm_levenberg.cpp
  g2o/core/optimization_algorithm_gauss_newton.cpp
  g2o/core/jacobian_workspace.cpp
  g2o/core/robust_kernel.cpp
  g2o/core/robust_kernel_factory.cpp
  g2o/core/robust_kernel_impl.cpp

  g2o/stuff/timeutil.cpp
  g2o/stuff/os_specific.c
  g2o/stuff/string_tools.cpp
  g2o/stuff/property.cpp
)

add_library(g2o SHARED ${G2O_SOURCES})

# Upstream g2o uses "../../config.h" from inside g2o/core/ etc. — point that to the
# generated header under <build>/g2o/config.h. Generated file lives at build/g2o/config.h
# and the source files live at <src>/g2o/core/..., so "../../config.h" from <src>/g2o/core/
# would resolve to <src>/config.h. Provide both source-root and build-root config.h locations
# via include dirs so the relative include lands on the generated file.
target_include_directories(g2o PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include>
)

# Generate a sibling config.h at the source root so "../../config.h" works during compile
configure_file(config.h.in ${CMAKE_CURRENT_SOURCE_DIR}/config.h)

target_link_libraries(g2o PUBLIC Eigen3::Eigen)

install(TARGETS g2o
  EXPORT export_g2o
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY g2o
  DESTINATION include
  FILES_MATCHING
    PATTERN "*.h"
    PATTERN "*.hpp"
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/config.h DESTINATION include)

ament_export_targets(export_g2o HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(eigen3_cmake_module Eigen3)

ament_package()
