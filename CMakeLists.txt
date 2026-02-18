cmake_minimum_required(VERSION 3.16)
project(gocator_driver)

set(CMAKE_CXX_STANDARD 17)

# Path to your Logan-Shi GO_SDK
set(GO_SDK /home/smith.15485/AA_DEVEL/gocator/src/GO_SDK)


# Find point cloud Library
FIND_PACKAGE(PCL 1.10 REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

# Add executable
add_executable(gocator_driver src/main.cpp)

# Include headers
include_directories(
    ${GO_SDK}/Gocator/GoSdk
    ${GO_SDK}/Platform/kApi
)

# Link libraries
target_link_libraries(gocator_driver
    ${GO_SDK}/lib/linux_x64/libGoSdk.so   # This contains GoSdk_Construct
    ${GO_SDK}/lib/linux_x64/libkApi.so
    ${PCL_LIBRARIES}
    pthread
)

# Install
install(TARGETS gocator_driver DESTINATION lib/${PROJECT_NAME})
