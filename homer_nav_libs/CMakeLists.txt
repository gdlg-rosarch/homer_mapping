cmake_minimum_required(VERSION 2.8.3)
project(homer_nav_libs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  tf
  cmake_modules
)

find_package(Eigen3 REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES homerExplorer homerMappingMath
	CATKIN_DEPENDS  roscpp geometry_msgs tf
	DEPENDS Eigen3)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(homerExplorer src/Explorer/Explorer.cpp)
add_library(homerMappingMath
	src/Math/Line2D.cpp
	src/Math/Transformation2D.cpp
	src/Math/Pose.cpp
	src/Math/Math.cpp
	src/Math/Point2D.cpp
)
