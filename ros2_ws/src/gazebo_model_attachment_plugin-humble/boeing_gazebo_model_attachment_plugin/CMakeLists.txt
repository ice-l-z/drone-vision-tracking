cmake_minimum_required(VERSION 3.8)
project(boeing_gazebo_model_attachment_plugin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(boeing_gazebo_model_attachment_plugin_msgs)

#
# Install python module and scripts
#
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

include_directories(include)

ament_export_dependencies(rosidl_default_runtime)

add_library(${PROJECT_NAME}_lib SHARED src/${PROJECT_NAME}.cpp)

target_include_directories(
    ${PROJECT_NAME}_lib PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
                            rclcpp
                            gazebo_ros
                            boeing_gazebo_model_attachment_plugin_msgs)

install(
    TARGETS ${PROJECT_NAME}_lib
    EXPORT ModelAttachment
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES
    DESTINATION include
)

# Add Tests
if(BUILD_TESTING)
    install(DIRECTORY test DESTINATION share/${PROJECT_NAME})
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(test/test_plugin.test.py)
endif()

ament_package()
