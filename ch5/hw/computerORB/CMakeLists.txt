project(ORB_feature)

cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories("/usr/include/eigen3")

add_executable(ORB_feature src/computeORB.cpp)
target_link_libraries(ORB_feature ${OpenCV_LIBS})
