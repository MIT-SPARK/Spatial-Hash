if(NOT TARGET gtest)
  include(FetchContent)
  FetchContent_Declare(
    googletest GIT_REPOSITORY https://github.com/google/googletest.git GIT_TAG v1.13.0
  )

  FetchContent_GetProperties(googletest)
  if(NOT googletest_POPULATED)
    FetchContent_Populate(googletest)
  endif()

  set(BUILD_SHARED_LIBS_THIS ${BUILD_SHARED_LIBS})
  set(BUILD_SHARED_LIBS OFF CACHE INTERNAL "")
  add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR} EXCLUDE_FROM_ALL)
  set(BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS_THIS} CACHE BOOL "")
endif()

include(GoogleTest)
enable_testing()

add_executable(
  utest_${PROJECT_NAME}
  utest_main.cpp
  utest_block.cpp
  utest_block_layer.cpp
  utest_grid.cpp
  utest_hash.cpp
  utest_layer.cpp
  utest_neighbor_utils.cpp
  utest_voxel_block.cpp
  utest_voxel_layer.cpp
)
target_include_directories(utest_${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(utest_${PROJECT_NAME} PRIVATE ${PROJECT_NAME} GTest::gtest_main)
gtest_add_tests(TARGET utest_${PROJECT_NAME})

install(TARGETS utest_${PROJECT_NAME}
RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
)
