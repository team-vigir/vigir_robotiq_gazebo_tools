cmake_minimum_required(VERSION 2.8.3)

project(vigir_robotiq_gazebo_tools)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs robotiq_s_model_control)

catkin_package(
   INCLUDE_DIRS 
   LIBRARIES fake_robotiq_publisher sim_robotiq_publisher sim_robotiq_gazebo_converter
   CATKIN_DEPENDS roscpp sensor_msgs robotiq_s_model_control
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(fake_robotiq_publisher src/fake_robotiq_pub.cpp )
target_link_libraries(fake_robotiq_publisher ${catkin_LIBRARIES})
add_dependencies(fake_robotiq_publisher ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

## Declare another cpp executable
add_executable(sim_robotiq_publisher src/sim_robotiq_pub.cpp )
target_link_libraries(sim_robotiq_publisher ${catkin_LIBRARIES})
add_dependencies(sim_robotiq_publisher ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

## Declare another cpp executable
add_executable(sim_robotiq_gazebo_converter src/sim_robotiq_gazebo_converter.cpp )
target_link_libraries(sim_robotiq_gazebo_converter ${catkin_LIBRARIES})
add_dependencies(sim_robotiq_gazebo_converter ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

#install executables
install( TARGETS
    fake_robotiq_publisher
    sim_robotiq_publisher
    sim_robotiq_gazebo_converter
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install the launch files.
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
  PATTERN "*~" EXCLUDE  
)

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
