cmake_minimum_required(VERSION 2.8.3)
project(grid_generator)  


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Grid_map_struct.msg
   Grid_map.msg
   OccupancyGrid.msg
   VoxelGrid.msg
   
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
 )
 


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ras_lab1_ds_calib
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(grid_generator_node src/grid_generator_node.cpp)
target_link_libraries(grid_generator_node  ${catkin_LIBRARIES})

add_dependencies(grid_generator_node  grid_generator_msgs_generate_messages_cpp)


add_executable(update_request src/update_request.cpp)
target_link_libraries(update_request ${catkin_LIBRARIES})

add_dependencies(update_request  update_request_msgs_generate_messages_cpp)

