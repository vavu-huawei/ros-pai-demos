cmake_minimum_required(VERSION 3.8)
project(pai_data_collection)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)

# Include directories
include_directories(include)

# Create the data_collector executable
add_executable(data_collector src/data_collector.cpp)

# Link libraries
target_link_libraries(data_collector
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  yaml-cpp
  ${rcutils_LIBRARIES}
)

# Install the executable
install(TARGETS
  data_collector
  DESTINATION lib/${PROJECT_NAME}
)

# Install config file
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
endif()

ament_package()
