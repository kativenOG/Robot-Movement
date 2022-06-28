cmake_minimum_required(VERSION 3.0.2)
project(robot_movement)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
message_generation
roscpp
std_msgs
) 

# Custom msg
add_message_files(
   FILES
   customMsg.msg
 )
generate_messages(
 DEPENDENCIES
   std_msgs
)

catkin_package( 
    CATKIN_DEPENDS message_runtime urdf roscpp message_generation std_msgs
)


find_package(Eigen3 REQUIRED)
find_package(gazebo_ros_link_attacher REQUIRED)


include_directories(
${catkin_INCLUDE_DIRS}
${Eigen_INCLUDE_DIRS}
${gazebo_ros_link_attacher_INCLUDE_DIRS}
)
# Include la cartella degli headers
include_directories(movement_include)

add_executable(mock_node mockPublisher.cpp)
add_executable(point_1_node punto1.cpp)
add_executable(point_2_node punto2.cpp)
# add_executable(point_3_node punto3.cpp)
# add_executable(point_4_node punto3.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries( 
    point_1_node 
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
)
target_link_libraries( 
    point_2_node
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
)
target_link_libraries( 
    mock_node
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
)


