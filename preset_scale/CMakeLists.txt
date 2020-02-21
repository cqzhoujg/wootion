cmake_minimum_required(VERSION 2.8.3)
project(preset_scale)

add_compile_options(-std=c++11)
add_definitions(-g)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs 
  geometry_msgs
  wootion_msgs
)
find_package(OpenCV REQUIRED)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include/${PROJECT_NAME}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS std_msgs
)

add_executable(preset_scale src/service_caller.cpp src/preset_scale_node.cpp src/ini_file.cpp src/preset_scale.cpp)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS} 
 )

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other useful stuff for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
