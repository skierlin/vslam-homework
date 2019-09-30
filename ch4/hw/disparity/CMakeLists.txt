cmake_minimum_required( VERSION 2.8 )
project(disparity)

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )

# 寻找OpenCV库
find_package( OpenCV 3 REQUIRED )
# 添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES("/usr/include/eigen3")

find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

add_executable( disparity src/disparity.cpp )
# 链接OpenCV库
target_link_libraries( disparity ${OpenCV_LIBS} )
target_link_libraries( disparity ${Pangolin_LIBRARIES} )
