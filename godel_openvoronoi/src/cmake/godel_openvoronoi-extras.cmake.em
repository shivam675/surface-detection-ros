@[if DEVELSPACE]@
list(APPEND @(PROJECT_NAME)_INCLUDE_DIRS @(CMAKE_CURRENT_SOURCE_DIR))
@[else]@
get_filename_component(@(PROJECT_NAME)_inc_temp ${@(PROJECT_NAME)_DIR}/../../../@(CATKIN_PACKAGE_INCLUDE_DESTINATION)/include REALPATH)
list(APPEND @(PROJECT_NAME)_INCLUDE_DIRS ${@(PROJECT_NAME)_inc_temp})

# this is necessary because headers in 'openvoronoi' itself are included without the 'openvoronoi' prefix
list(APPEND @(PROJECT_NAME)_INCLUDE_DIRS ${@(PROJECT_NAME)_inc_temp}/openvoronoi)

unset(@(PROJECT_NAME)_inc_temp)
@[end if]@
