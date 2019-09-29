cmake_minimum_required( VERSION 2.8 )
project( hw )

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

add_executable( hw2_2 src/hw2_2.cpp )
add_executable( hw2_3 src/hw2_3.cpp )
