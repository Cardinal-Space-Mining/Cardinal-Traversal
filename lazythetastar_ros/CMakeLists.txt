
cmake_minimum_required(VERSION 3.9.4)
project(lazythetastar_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

# Specify include directories
include_directories(
  include
  ${lazythetastar_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node main.cpp)

ament_target_dependencies(${PROJECT_NAME}_node 
  rclcpp 
  nav_msgs 
  geometry_msgs 
  std_msgs 
  lazythetastar)

target_include_directories(${PROJECT_NAME}_node
  PRIVATE 
    include
)

target_link_libraries(${PROJECT_NAME}_node
  lazythetastar
)

# Check for IPO / LTO support
include(CheckIPOSupported)
check_ipo_supported(RESULT supported OUTPUT error)

if(supported)
  message(STATUS "IPO / LTO enabled")
  set_property(TARGET ${PROJECT_NAME}_node PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
else()
  message(STATUS "IPO / LTO not supported: <${error}>")
endif()

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
