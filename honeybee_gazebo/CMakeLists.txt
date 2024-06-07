cmake_minimum_required(VERSION 3.8)
project(honeybee_gazebo)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY configs launch worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
