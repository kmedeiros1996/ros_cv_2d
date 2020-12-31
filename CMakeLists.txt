cmake_minimum_required(VERSION 3.5)
project(ros_cv_2d)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cxxopts CONFIG REQUIRED)

include_directories(include)
add_library(corner_extractor_driver SHARED
            src/corner_extractor_driver.cc)
target_compile_definitions(corner_extractor_driver
  PRIVATE "CORNER_EXTRACTOR_DLL")
ament_target_dependencies(corner_extractor_driver rclcpp rclcpp_components OpenCV sensor_msgs cv_bridge)

add_executable(corner_extractor_node src/corner_extractor_node.cc)
target_link_libraries(corner_extractor_node corner_extractor_driver)
ament_target_dependencies(corner_extractor_node
  rclcpp)

install(TARGETS
  corner_extractor_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  corner_extractor_node
  DESTINATION lib/${PROJECT_NAME})


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
