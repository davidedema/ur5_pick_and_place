cmake_minimum_required(VERSION 3.0.2)
project(communication)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs)
find_package(Eigen3 3.3 REQUIRED)

#add_message_files(
#  FILES
#  dest_point.msg
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  sensor_msgs
#)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
add_executable(publisher src/publisher.cpp)
add_executable(newpublisher src/newpublisher.cpp)
target_link_libraries(publisher ${catkin_LIBRARIES})
install(TARGETS publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
target_link_libraries(newpublisher ${catkin_LIBRARIES})
install(TARGETS newpublisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

