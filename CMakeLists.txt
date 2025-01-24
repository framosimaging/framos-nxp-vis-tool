cmake_minimum_required(VERSION 3.10)
project(DisplayImageApp)

# Specify the C++ standard NXP uses this version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include the header directory
include_directories(include config)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Add the executable
add_executable(display_image src/display_image.cpp src/v4l2_subdevice_controls.cpp)

# Link OpenCV libraries
target_include_directories(display_image PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(display_image PRIVATE ${OpenCV_LIBS} g2d)

# Print the OpenCV version
message(STATUS "Using OpenCV version ${OpenCV_VERSION}")