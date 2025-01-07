cmake_minimum_required(VERSION 3.5)
project(multi_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Add libraries
add_library(tester_node SHARED src/tester.cpp)

# Create Cpp executable
add_executable(tester src/tester_main.cpp)

# Target dependencies
set(tester_dependencies rclcpp std_msgs)
ament_target_dependencies(tester_node ${tester_dependencies})
ament_target_dependencies(tester ${tester_dependencies})

# Link libraries
target_link_libraries(tester tester_node)

# Install Cpp executables
install(TARGETS
  tester
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  tester_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
