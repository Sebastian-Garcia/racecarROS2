cmake_minimum_required(VERSION 3.5)
project(ackermann_cmd_mux)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# Nodelet library
add_library(${PROJECT_NAME} SHARED src/ackermann_cmd_mux.cpp)
ament_target_dependencies(${PROJECT_NAME}
  ackermann_msgs
  rcl_interfaces
  rclcpp
  rclcpp_components
  rcpputils
  std_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "ackermann_cmd_mux::AckermannCmdMux"
  EXECUTABLE ackermann_cmd_mux_node)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ros_testing REQUIRED)

  add_ros_test(test/test_ackermann_cmd_mux.py
    TARGET test_ackermann_cmd_mux
    TIMEOUT 30
  )

  add_ros_test(test/test_ackermann_cmd_mux_updates.py
    TARGET test_ackermann_cmd_mux_updates
    TIMEOUT 30
  )
endif()

ament_package()
