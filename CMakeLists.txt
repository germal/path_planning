cmake_minimum_required(VERSION 2.8.3)

project(path_planning)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    geometry_msgs
    message_generation
)

include_directories(${catkin_INCLUDE_DIRS})

add_message_files(
  FILES
  path.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
  CATKIN_DEPENDS geometry_msgs
)



add_executable(${PROJECT_NAME}_node src/listener.cpp)


target_link_libraries(${PROJECT_NAME}_node
    ${CATKIN_LIBRARIES}
)
