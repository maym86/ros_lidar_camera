cmake_minimum_required(VERSION 2.8.3)
project(lidar_camera)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  pcl_ros
  image_geometry
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package( OpenCV 3 REQUIRED )
find_package(Ceres REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_camera
#  CATKIN_DEPENDS roscpp std_msg
#  DEPENDS system_lib
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/lidar_camera_node.cpp
        src/colormap.cpp
        src/utils.cpp
        src/process_lidar.cpp
        src/calibration_solver.cpp
        src/process_camera.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS} 
   ${Boost_LIBRARIES}
   ${CERES_LIBRARIES}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/test_lidar_camera.cpp)
if(TARGET ${PROJECT_NAME}-test)
	target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()
