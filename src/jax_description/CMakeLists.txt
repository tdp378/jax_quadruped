cmake_minimum_required(VERSION 3.8)
project(jax_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# This is the critical part!
install(
  DIRECTORY
    launch
    urdf
    meshes
    world
  DESTINATION share/${PROJECT_NAME}
)

ament_package()