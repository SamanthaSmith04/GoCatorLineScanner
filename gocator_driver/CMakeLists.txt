cmake_minimum_required(VERSION 3.16)
project(gocator_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ROS2
find_package(ament_cmake REQUIRED)

# External dependencies
find_package(PCL 1.10 REQUIRED)

# Path to GO_SDK
set(GO_SDK /home/smith.15485/AA_DEVEL/gocator/src/GO_SDK)

# =========================
# Build library from gocator_commands.cpp
# =========================
add_library(${PROJECT_NAME} SHARED
    src/gocator_commands.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${GO_SDK}/Gocator/GoSdk
    ${GO_SDK}/Platform/kApi
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    ${GO_SDK}/lib/linux_x64/libGoSdk.so
    ${GO_SDK}/lib/linux_x64/libkApi.so
    ${PCL_LIBRARIES}
    pthread
)

# =========================
# Install library and headers
# =========================
install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}   # optional
)

install(DIRECTORY include/
    DESTINATION include
)

# =========================
# Export library for other ROS2 packages
# =========================
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()