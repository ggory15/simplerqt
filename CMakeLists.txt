cmake_minimum_required(VERSION 3.0.2)
project(talos_wbc_gui)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  rqt_gui
  rqt_gui_cpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  tf
)

FIND_PACKAGE(Eigen3 REQUIRED) 
find_package(Qt5Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)
catkin_python_setup()

roslint_cpp()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp sensor_msgs std_msgs geometry_msgs nav_msgs tf
)

set(talos_wbc_gui_SRCS
  src/talos_wbc_gui.cpp
)

set(talos_wbc_gui_HDRS
  include/talos_wbc_gui/talos_wbc_gui.h
)

set(talos_wbc_gui_UIS
  src/talos_wbc_gui.ui
)

qt5_wrap_cpp(talos_wbc_gui_MOCS ${talos_wbc_gui_HDRS})
qt5_wrap_ui(talos_wbc_gui_UIS_H ${talos_wbc_gui_UIS})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  ${talos_wbc_gui_SRCS}
  ${talos_wbc_gui_MOCS}
  ${talos_wbc_gui_UIS_H}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}  
  Qt5::Widgets
)

# target_link_libraries(${PROJECT_NAME} Qt5::Widgets)