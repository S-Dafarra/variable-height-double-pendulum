find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_msgs REQUIRED)

add_subdirectory(src)

if(BUILD_TESTING)
    add_subdirectory(test)
endif()
