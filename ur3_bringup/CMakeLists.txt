cmake_minimum_required(VERSION 3.8)
project(ur3_bringup)

find_package(ament_cmake REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py"
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md README.zh-TW.md DESTINATION share/${PROJECT_NAME})

ament_package()
