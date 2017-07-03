cmake_minimum_required(VERSION 2.8.3)
project(rosbuzz2)

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++11")
endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mavros_msgs
  sensor_msgs
  message_generation
)

##############################
##############################

add_message_files(
 FILES
 neigh_pos.msg
)

generate_messages(
DEPENDENCIES
sensor_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs mavros_msgs sensor_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
  include ${rosbuzz2_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Executables 
add_executable(rosbuzz2_node src/rosbuzz.cpp
  src/roscontroller.cpp
  src/Types/position.cpp
  src/RosBuzz_Abstraction_Layer/ros_callback_interface.cpp
  src/RosBuzz_Abstraction_Layer/rci_current_position.cpp
  src/RosBuzz_Implementation_Layer/rosbuzz_current_position_impl.cpp
  src/Buzz_Abstraction_Layer/buzz_utility.cpp
  src/Buzz_Abstraction_Layer/buzz_vm_interface.cpp
  src/Buzz_Abstraction_Layer/buzz_update_closure.cpp
  )
target_link_libraries(rosbuzz2_node ${catkin_LIBRARIES} buzz buzzdbg pthread)
add_dependencies(rosbuzz2_node rosbuzz2_generate_messages_cpp)

# Executables and libraries for installation to do
install(TARGETS rosbuzz2_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

find_package(catkin REQUIRED COMPONENTS roslaunch)
roslaunch_add_file_check(launch)
