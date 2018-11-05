cmake_minimum_required(VERSION 3.0)
project(pv_estimator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
  roscpp
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
#set(EIGEN3_INCLUDE_DIRS /usr/share/eigen)

add_message_files(DIRECTORY ${PROJECT_SOURCE_DIR}/msg FILES
  State.msg
  Meas.msg
)

generate_messages( DEPENDENCIES
  std_msgs
  geometry_msgs 
)

catkin_package(CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  roscpp
  message_runtime
)

#########
# BUILD #
#########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(estimator_node
               src/estimator_node.cpp
               src/simulator.cpp
               src/params.cpp
               src/estimator.cpp)
add_executable(simulator_node
               src/simulator_node.cpp
               src/simulator.cpp
               src/params.cpp)

set(EXEC_LIST estimator_node)
foreach(EXEC_FILE ${EXEC_LIST})
  target_link_libraries(${EXEC_FILE} ${catkin_LIBRARIES} Eigen3::Eigen)
  add_dependencies(${EXEC_FILE} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
  install(TARGETS ${EXEC_FILE}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach(EXEC_FILE ${EXEC_LIST})
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
