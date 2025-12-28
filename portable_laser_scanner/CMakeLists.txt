cmake_minimum_required(VERSION 3.8)
project(portable_laser_scanner)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 17)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Only strictly required build dependency
find_package(ament_cmake REQUIRED)

# 1. Install Python Scripts directly as executables
# This copies them to lib/motor_test_description/ where 'ros2 run' looks for them
install(PROGRAMS
  scripts/motor_angle_control.py
  scripts/motor_encoder_sim.py
  scripts/motor_rpm_control.py
  scripts/motor_sim_node.py
  scripts/rpm_to_velocity.py
  scripts/robot_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# 2. Install Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 3. Install URDF files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

ament_package()
