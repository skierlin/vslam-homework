cmake_minimum_required(VERSION 2.8)
project(ICP)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
)

add_executable(ICP src/ICP.cpp)
target_link_libraries(ICP
        ${Pangolin_LIBRARIES}
        ${Sophus_LIBRARIES})
