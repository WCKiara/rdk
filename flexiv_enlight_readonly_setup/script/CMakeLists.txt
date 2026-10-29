# script/ 下的 C++ 程序，链接项目自带的 RDK 2.1（.rdk-v2.1）。在项目根目录编译：
#   cmake -S script -B .build/script
#   cmake --build .build/script
cmake_minimum_required(VERSION 3.22.1)
project(flexiv_script LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()

set(flexiv_rdk_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.rdk-v2.1/lib/cmake/flexiv_rdk" CACHE PATH "RDK 2.1 CMake config")
find_package(flexiv_rdk 2.1 EXACT REQUIRED)

add_executable(arm1_transparent_arm2_hold_rt arm1_transparent_arm2_hold_rt.cpp)
target_link_libraries(arm1_transparent_arm2_hold_rt flexiv::flexiv_rdk)
set_target_properties(arm1_transparent_arm2_hold_rt PROPERTIES CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON)
