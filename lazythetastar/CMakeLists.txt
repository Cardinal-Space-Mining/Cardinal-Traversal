cmake_minimum_required(VERSION 3.10)
project(lazythetastar VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOURCE_LIST
    "${PROJECT_SOURCE_DIR}/src/node_grid.cpp")

set(HEADER_LIST
    "${PROJECT_SOURCE_DIR}/include/node_grid.hpp"
    "${PROJECT_SOURCE_DIR}/include/grid_utils.hpp"
    )

add_library(${PROJECT_NAME} SHARED ${SOURCE_LIST})

# Enable debug tools
target_compile_definitions(${PROJECT_NAME} PUBLIC DEBUG_TOOLS)

# Find dependencies
find_package(OpenCV REQUIRED)

# Include directories
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${OpenCV_INCLUDE_DIRS}  # Add OpenCV include directories
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        ${OpenCV_LIBS}  # Link against OpenCV libraries
)

install(DIRECTORY include/
    DESTINATION include
)

# IDEs should put the headers in a nice place
source_group(TREE "${PROJECT_SOURCE_DIR}/include" PREFIX "Header Files" FILES ${HEADER_LIST})

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
        DESTINATION include)

install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
)
