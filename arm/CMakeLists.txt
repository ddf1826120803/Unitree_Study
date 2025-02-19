cmake_minimum_required(VERSION 3.16)
project(g1_dual_arm)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

add_executable(g1_dual_arm g1_dual_arm_example.cpp)
target_link_libraries(g1_dual_arm  unitree_sdk2)

SET(CMAKE_BUILD_TYPE Release)