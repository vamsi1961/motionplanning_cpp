cmake_minimum_required(VERSION 2.8)
project( Planning )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( a_star A_star/a_star.cpp )
target_link_libraries( a_star ${OpenCV_LIBS} )

add_executable( DisplayImage DisplayImage.cpp )
target_link_libraries( DisplayImage ${OpenCV_LIBS} )
