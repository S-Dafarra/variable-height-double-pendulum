add_subdirectory(core)

option(BUILD_INTERFACE "Build ROS2 nodes for communication" ON)
if (BUILD_INTERFACE)
    add_subdirectory(interface)
endif()
