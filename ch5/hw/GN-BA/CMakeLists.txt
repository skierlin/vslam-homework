cmake_minimum_required(VERSION 2.8)
project(GN_BA)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${Sophus_INCLUDE_DIRS}
)

add_executable(GN_BA src/GN-BA.cpp)
target_link_libraries(GN_BA
        ${Sophus_LIBRARIES})
