cmake_minimum_required(VERSION 3.8)
project(honeybee_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(DIRECTORY
  urdf
  meshes
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(xacro urdf)
ament_package()
