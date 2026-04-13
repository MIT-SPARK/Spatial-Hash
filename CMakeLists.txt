cmake_minimum_required(VERSION 3.22)
project(spatial_hash VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  # from: https://www.kitware.com//cmake-and-the-default-build-type/
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()

if(NOT CMAKE_INTERPROCEDURAL_OPTIMIZATION)
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON CACHE STRING "Turn on -flto" FORCE)
endif()

option(BUILD_SHARED_LIBS "Build shared libs" ON)

find_package(Eigen3 REQUIRED)

add_library(
  ${PROJECT_NAME}
  src/block.cpp
  src/grid.cpp
  src/neighbor_utils.cpp
)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>
                         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

include(CTest)
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(tests)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE 1)
add_library(
  spatial_hash::${PROJECT_NAME} ALIAS ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME} EXPORT spatial_hash-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT spatial_hash-targets FILE spatial_hashTargets.cmake NAMESPACE spatial_hash::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/spatial_hash
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/spatial_hashConfigVersion.cmake VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/spatial_hashConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/spatial_hashConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/spatial_hash
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/spatial_hashConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/spatial_hashConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/spatial_hash
)
