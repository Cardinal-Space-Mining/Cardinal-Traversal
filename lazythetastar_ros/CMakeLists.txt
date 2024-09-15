cmake_minimum_required(VERSION 3.9.4)
project(lazythetastar_ros)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Other dependencies
find_package(OpenCV REQUIRED)
find_package(lazythetastar REQUIRED)

# Add executable for ROS node
add_executable(${PROJECT_NAME} src/main.cpp)

ament_target_dependencies(${PROJECT_NAME} rclcpp nav_msgs geometry_msgs std_msgs lazythetastar)

target_include_directories(${PROJECT_NAME} PRIVATE ${lazythetastar_INCLUDE_DIRS})
message(INCLUDE_DIR: ${lazythetastar_INCLUDE_DIRS})

# Link libraries
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} lazythetastar)

# Check for IPO / LTO support
include(CheckIPOSupported)
check_ipo_supported(RESULT supported OUTPUT error)

if(supported)
  message(STATUS "IPO / LTO enabled")
  set_property(TARGET ${PROJECT_NAME} PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
else()
  message(STATUS "IPO / LTO not supported: <${error}>")
endif()

# Set compile features
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# Install the node and other resources
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
