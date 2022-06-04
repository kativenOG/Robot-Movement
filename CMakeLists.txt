cmake_minimum_required(VERSION 3.0.2)
project(robot_movement)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
) 
catkin_package( 
    CATKIN_DEPENDS urdf 
)
find_package(Eigen3 REQUIRED)
find_package(gazebo_ros_link_attacher REQUIRED)

include_directories(
${catkin_INCLUDE_DIRS}
${Eigen_INCLUDE_DIRS}
${gazebo_ros_link_attacher_INCLUDE_DIRS}
)
include_directories(movement_include)

# add_executable(${PROJECT_NAME}_node1 robot_movement_node.cpp)
add_executable(point_1_node punto1.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries( #${PROJECT_NAME}_node1
    point_1_node
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
)

