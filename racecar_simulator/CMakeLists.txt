cmake_minimum_required(VERSION 3.5)
project(racecar_simulator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Set C++ standard to 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


######################################
# Compile the library
######################################

# Add includes
include_directories(include)
file(GLOB SRC_FILES src/*.cpp)
add_library(${PROJECT_NAME} ${SRC_FILES})

# Install the library to CMAKE_INSTALL_PREFIX
# which defaults to /usr/local
install(TARGETS ${PROJECT_NAME}
        DESTINATION lib)
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Include directories
include_directories(include
${rclcpp_INCLUDE_DIRS}
${tf2_INCLUDE_DIRS}
${tf2_ros_INCLUDE_DIRS}
${tf2_geometry_msgs_INCLUDE_DIRS}
${ackermann_msgs_INCLUDE_DIRS}
${sensor_msgs_INCLUDE_DIRS}
${geometry_msgs_INCLUDE_DIRS})

# Add the library to ament package
ament_target_dependencies(${PROJECT_NAME}
      rclcpp
      tf2
      tf2_ros
      tf2_geometry_msgs
      ackermann_msgs
      sensor_msgs
      geometry_msgs)

# Add the nodes
file(GLOB NODE_SRC_FILES node/*.cpp)
foreach(_node_file ${NODE_SRC_FILES})
get_filename_component(_node_name ${_node_file} NAME_WE)
add_executable(${_node_name} ${_node_file})
ament_target_dependencies(${_node_name} ${PROJECT_NAME})
endforeach()

# Install the library
install(TARGETS ${PROJECT_NAME}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)
install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION include/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}/
)

# Include map files
install(DIRECTORY maps
DESTINATION share/${PROJECT_NAME}/
)


# Set the source and destination paths
set(SHARE_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME})

# Install Xacro files to the share directory
install(
  DIRECTORY share/${PROJECT_NAME}/
  DESTINATION ${SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xacro"
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
