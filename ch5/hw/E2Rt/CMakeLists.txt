project(E2Rt)

cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package( Sophus REQUIRED )
include_directories(
	${EIGEN3_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIR}
        ${Sophus_INCLUDE_DIRS})


add_executable(E2Rt src/E2Rt.cpp)
target_link_libraries( E2Rt ${Sophus_LIBRARIES} )
