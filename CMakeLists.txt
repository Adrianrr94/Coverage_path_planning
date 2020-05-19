cmake_minimum_required(VERSION 2.8.3)
project(zig_zag_trajectory)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  roslint
)

find_package(CGAL REQUIRED COMPONENTS Core)

include( ${CGAL_USE_FILE} )
include( CGAL_CreateSingleSourceCGALProgram )

add_service_files(
  FILES
  Torres16.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# lint
roslint_cpp()
roslint_python()

# Executables
add_executable(zig_zag_trajectory src/torres_etal_2016.cpp)

add_dependencies(zig_zag_trajectory ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(zig_zag_trajectory ${catkin_LIBRARIES})

target_link_libraries(zig_zag_trajectory ${catkin_LIBRARIES})
