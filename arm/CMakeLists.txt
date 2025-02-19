cmake_minimum_required(VERSION 3.16)
project(g1_arm)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

add_executable(g1_arm7_example g1_arm7_sdk_dds_example.cpp)
target_link_libraries(g1_arm7_example  unitree_sdk2)

SET(CMAKE_BUILD_TYPE Release)