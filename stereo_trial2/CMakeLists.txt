cmake_minimum_required(VERSION 2.8)
project( stereo_camera_calib2 )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( stereo_camera_calib2 stereo_camera_calib2.cpp )
target_link_libraries( stereo_camera_calib2 ${OpenCV_LIBS} )
