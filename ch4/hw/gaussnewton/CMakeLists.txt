cmake_minimum_required( VERSION 2.8 )
project(gaussnewton)

find_package( OpenCV 3 REQUIRED )
# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories("/usr/include/eigen3")

add_executable( gaussnewton src/gaussnewton.cpp )
target_link_libraries( gaussnewton ${OpenCV_LIBS} )
