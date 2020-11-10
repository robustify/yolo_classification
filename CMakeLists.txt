cmake_minimum_required(VERSION 2.8.3)
project(yolo_classification)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  dynamic_reconfigure
  cv_bridge
  darknet_ros
)
find_package(OpenCV REQUIRED)

if("$ENV{ROS_DISTRO}" STRGREATER "melodic")
  set(CMAKE_CXX_STANDARD 17)
else()
  set(CMAKE_CXX_STANDARD 11)
endif()

generate_dynamic_reconfigure_options(
  cfg/Darknet.cfg
)

catkin_package(
  DEPENDS OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/yolo_classification.cpp
  src/YoloClassification.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

install(
  TARGETS
    ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
