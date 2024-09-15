cmake_minimum_required(VERSION 3.10)
project(lazythetastar VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)

set(SOURCE_LIST
    "${PROJECT_SOURCE_DIR}/src/node_grid.cpp")

set(HEADER_LIST
    "${PROJECT_SOURCE_DIR}/include/node_grid.hpp"
    "${PROJECT_SOURCE_DIR}/include/grid_utils.hpp")

add_library(${PROJECT_NAME} SHARED ${SOURCE_LIST} ${HEADER_LIST})

# Find dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} Eigen3::Eigen)
target_include_directories(${PROJECT_NAME} PRIVATE ${EIGEN3_INCLUDE_DIRS})

set_target_properties(${PROJECT_NAME}
    PROPERTIES
        VERSION ${PROJECT_VERSION}
        SOVERSION ${PROJECT_VERSION_MAJOR}
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME}
	PUBLIC 
		cxx_std_17
)

source_group(TREE "${PROJECT_SOURCE_DIR}/include" PREFIX "Header Files" FILES ${HEADER_LIST})

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/lazythetastarConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/lazythetastarConfig.cmake"
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/lazythetastarConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(DIRECTORY include/ 
    DESTINATION include)

install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/lazythetastarConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/lazythetastarConfigVersion.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
)
