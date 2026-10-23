cmake_minimum_required(VERSION 3.8)
project(ur3_bringup)

find_package(ament_cmake REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/start_separate_terminals.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
