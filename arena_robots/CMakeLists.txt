cmake_minimum_required(VERSION 3.5)
project(arena_robots)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY robots/
  DESTINATION share/${PROJECT_NAME}/robots
)

install(
  DIRECTORY components/
  DESTINATION share/${PROJECT_NAME}/components
  PATTERN ".git*" EXCLUDE
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/task_server DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/goal_to_plan_bridge DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/power_publisher DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/acoustics_publisher DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_pytest tests
    TIMEOUT 120
  )
endif()

ament_package()