cmake_minimum_required(VERSION 3.5)
# add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

project(lvxReader)

set(PCL_DIR "/usr/local/include/pcl-1.9")
find_package(PCL 1.9 REQUIRED COMPONENTS common io visualization)
include_directories(${PCL_USE_FILE})

set(VTK_DIR "/usr/local/include/vtk-8.2")
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_executable(lvxReader lvx_reader.cpp)

target_link_libraries(lvxReader ${PCL_LIBRARIES})
target_link_libraries(lvxReader ${VTK_LIBRARIES})