cmake_minimum_required(VERSION 3.5)
project(leptrino_force_torque)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}
  src/force_torque_sensor_node.cpp
  src/force_torque_sensor.cpp
  src/rs_comm.cpp
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "leptrino::ForceTorqueSensorNode"
)

ament_auto_add_executable(${PROJECT_NAME}_node
  src/force_torque_sensor_node_main.cpp
)

install(DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
