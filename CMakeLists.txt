cmake_minimum_required(VERSION 3.5)
project(yost_imu_driver)

find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/YostLabDriver.cpp,
  src/YostLabDriverNode.cpp
)

ament_package()
