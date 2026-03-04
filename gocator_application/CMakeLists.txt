cmake_minimum_required(VERSION 3.19)
project(gocator_application LANGUAGES CXX)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(bt_common_nodes)
find_package(gocator_driver REQUIRED)
find_package(PCL REQUIRED)

find_package(MPI REQUIRED)
if(NOT TARGET MPI::MPI_C)
  add_library(MPI::MPI_C INTERFACE IMPORTED)
  set_target_properties(MPI::MPI_C PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${MPI_C_INCLUDE_PATH}"
    INTERFACE_LINK_LIBRARIES "${MPI_C_LIBRARIES}"
  )
endif()
# Qt configuration
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Source files
set(UIS
    src/gocator.ui
)

qt5_wrap_ui(UI_HEADERS ${UIS})

set(SRCS ${UI_HEADERS}
    src/gocator_widget.cpp
    include/gocator_widget.hpp
)

# Common dependencies for all targets
set(COMMON_DEPS
    ament_index_cpp
    behaviortree_cpp
    bt_common_nodes
    gocator_driver
)

# Shared library
add_library(gocator_qt_gui_lib SHARED ${SRCS} ${UIS})

target_include_directories(gocator_qt_gui_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

ament_target_dependencies(gocator_qt_gui_lib ${COMMON_DEPS})

target_link_libraries(gocator_qt_gui_lib
    Qt5::Widgets
    ${PCL_LIBRARIES}
)

# Main executable
add_executable(gocator_qt_gui src/main.cpp)
target_link_libraries(gocator_qt_gui gocator_qt_gui_lib)

# Installation
install(TARGETS 
    gocator_qt_gui
    gocator_qt_gui_lib
    EXPORT ${PROJECT_NAME}_targets
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Package export
ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${COMMON_DEPS} Qt5 PCL)

ament_package()
