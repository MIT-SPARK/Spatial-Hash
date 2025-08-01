find_package(GTest REQUIRED)

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
