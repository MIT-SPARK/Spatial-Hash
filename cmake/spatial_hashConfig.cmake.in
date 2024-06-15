get_filename_component(spatial_hash_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

find_dependency(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED IMPORTED_TARGET libglog)

if(NOT TARGET spatial_hash::spatial_hash)
  include("${spatial_hash_CMAKE_DIR}/spatial_hashTargets.cmake")
endif()

set(spatial_hash_LIBRARIES spatial_hash::spatial_hash)
