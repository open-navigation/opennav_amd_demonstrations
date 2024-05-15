cmake_minimum_required(VERSION 3.8)
project(honeybee_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/install
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
