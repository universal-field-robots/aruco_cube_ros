cmake_minimum_required(VERSION 2.8.3)
project(aruco_cube_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  dynamic_reconfigure
  message_generation
  nodelet
)

add_service_files(
  FILES
  GetPose.srv
)

# GENERATE MESSAGE FILES
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/ArucoCube.cfg
)

find_package(OpenCV 3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES aruco_cube_nodelet
  #CATKIN_DEPENDS message_runtime geometry_msgs std_msgs tf2_ros tf2 nodelet
)

# ADD NODES
add_executable(aruco_cube_node src/aruco_cube_node.cpp)
target_link_libraries(aruco_cube_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES})
add_dependencies(aruco_cube_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

# ADD NODELET LIBRARIES
add_library(aruco_cube_nodelet src/aruco_cube_nodelet.cpp)
target_link_libraries(aruco_cube_nodelet ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES})
add_dependencies(aruco_cube_nodelet ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(TARGETS aruco_cube_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS aruco_cube_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)
