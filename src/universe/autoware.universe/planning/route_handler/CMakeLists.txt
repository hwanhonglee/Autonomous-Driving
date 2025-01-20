cmake_minimum_required(VERSION 3.14)
project(route_handler)

#HH_240105
add_compile_options(-Wno-error=unused-local-typedefs)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(route_handler SHARED
  src/route_handler.cpp
)

ament_auto_package()
