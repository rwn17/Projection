cmake_minimum_required( VERSION 2.8 )
project( projection )

# Debug mode 
set( CMAKE_BUILD_TYPE "Debug")
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# opencv 
find_package( OpenCV 3.2.0 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# pcl 
find_package( PCL REQUIRED COMPONENT common io )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

add_executable( projection projection.cpp )
target_link_libraries( projection ${OpenCV_LIBS} ${PCL_LIBRARIES} )
