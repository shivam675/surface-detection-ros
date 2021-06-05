cmake_minimum_required(VERSION 2.8...3.13)
project(godel_openvoronoi)

find_package(catkin REQUIRED)

find_package(Boost REQUIRED graph)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(qd REQUIRED)
set(qd_INCLUDE_DIRS ${QD_INCLUDE_DIR})
set(qd_LIBRARIES ${QD_LIBRARY})

set(OVD_SRC
  voronoidiagram.cpp
  vertex.cpp
  edge.cpp
  checker.cpp
  common/point.cpp
  vertex_positioner.cpp
  common/numeric.cpp
  medial_axis_pocket.cpp
  medial_axis_walk.cpp
  offset.cpp
)

set(OVD_INCLUDE_FILES
  graph.hpp
  voronoidiagram.hpp
  vertex.hpp
  edge.hpp
  site.hpp
  checker.hpp
  vertex_positioner.hpp
  kdtree.hpp
  offset.hpp
  offset_sorter.hpp
  filter.hpp
  medial_axis_filter.hpp
  medial_axis_walk.hpp
  medial_axis_pocket.hpp
  polygon_interior_filter.hpp
  island_filter.hpp
  version.hpp
  ${CMAKE_CURRENT_BINARY_DIR}/version_string.hpp
)

set(OVD_INCLUDE_COMMON_FILES
  common/numeric.hpp
  common/point.hpp
  common/halfedgediagram.hpp
)

set( OVD_INCLUDE_UTIL_FILES
  utility/vd2svg.hpp
  utility/simple_svg_1.0.0.hpp
)

set( OVD_INCLUDE_SOLVERS_FILES
  solvers/solution.hpp
  solvers/solver.hpp
  solvers/solver_lll.hpp
  solvers/solver_lll_para.hpp
  solvers/solver_ppp.hpp
  solvers/solver_qll.hpp
  solvers/solver_sep.hpp
  solvers/solver_alt_sep.hpp
)

set_source_files_properties(
  ${OVD_SRC}
  PROPERTIES COMPILE_FLAGS -Werror -Wall -Wundef -Wno-error=uninitialized -Wshadow  -Wno-long-long -Wno-deprecated -pedantic -pedantic-errors)

INCLUDE(version_string.cmake)
set(MY_VERSION "${GIT_MAJOR_VERSION}.${GIT_MINOR_VERSION}.${GIT_PATCH_VERSION}" CACHE STRING "version")

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${qd_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

catkin_package(
  # openvoronoi's own include dir will be appended by the 'extras' file exported below
  INCLUDE_DIRS ${qd_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  # not using DEPENDS here, as our library is already linked to libqd
  CFG_EXTRAS godel_openvoronoi-extras.cmake
)

add_library(${PROJECT_NAME} ${OVD_SRC})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${qd_LIBRARIES})


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES ${OVD_INCLUDE_FILES}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/include/openvoronoi
)

install(FILES ${OVD_INCLUDE_COMMON_FILES}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/include/openvoronoi/common
)

install(FILES ${OVD_INCLUDE_UTIL_FILES}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/include/openvoronoi/utility
)

install(FILES ${OVD_INCLUDE_SOLVERS_FILES}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/include/openvoronoi/solvers
)
