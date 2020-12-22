cmake_minimum_required(VERSION 3.5)
project(petra_patient_monitoring)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(openpose_ros_msgs REQUIRED)
find_package(petra_core REQUIRED)
find_package(petra_interfaces REQUIRED)
#find_package(sensor_msgs REQUIRED)
#find_package(cv_bridge REQUIRED)
#find_package(image_transport REQUIRED)
#find_package(OpenCV REQUIRED)

# for python code
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

include_directories(
    include
    #${OpenCV_INCLUDE_DIRS}
    ${petra_core_INCLUDE_DIRS}
)

set(DEPENDENCIES 
    rclcpp
    openpose_ros_msgs
    petra_core
    petra_interfaces
)

add_executable(FeatureExtractor
    src/feature_extractor.cpp
)

ament_target_dependencies(FeatureExtractor
    ${DEPENDENCIES}
)

target_link_libraries(FeatureExtractor
#    ${OpenCV_LIBS}
    petra_core::petra_core_lib
)

# Install launch files
#install(DIRECTORY
#    launch
#    DESTINATION share/${PROJECT_NAME}/
#)

install(TARGETS
  FeatureExtractor
  DESTINATION lib/${PROJECT_NAME}
)

# for python code
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/collect_training_data.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
