cmake_minimum_required(VERSION 3.8)
project(slam_ultrasonic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
    DIRECTORY src launch config
    DESTINATION lib/${PROJECT_NAME}
)

install( 
  DIRECTORY launch src config
  DESTINATION share/${PROJECT_NAME}/
)

install(
    PROGRAMS
        src/servo_node.py
        src/ultrasonic_node.py
        src/cmd_vel_to_joints.py
        src/obstacle_avoidance_node.py
    DESTINATION lib/${PROJECT_NAME}
)
ament_package()
