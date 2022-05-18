cmake_minimum_required(VERSION 3.5)
project(notspot_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(notspot_description REQUIRED)
 
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()