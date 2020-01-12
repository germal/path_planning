cmake_minimum_required(VERSION 2.8.3)

project(path_planning)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
    nav_msgs
    message_generation
)

add_message_files(
  FILES
  path.msg
)

generate_messages(
    DEPENDANCIES
    geometry_msgs
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime 
    CATKIN_DEPENDS geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node test/a_star_utest.cpp)


target_link_libraries(${PROJECT_NAME}_node
    ${CATKIN_LIBRARIES}
)
