
set(EXECUTABLE_TARGET_NAME ${PROJECT_NAME}_interface)

# List of CPP (source) library files.
set(${EXECUTABLE_TARGET_NAME}_SRC
        src/main.cpp
)

# List of HPP (header) library files.
set(${EXECUTABLE_TARGET_NAME}_HDR
)


add_executable(${EXECUTABLE_TARGET_NAME} ${${EXECUTABLE_TARGET_NAME}_SRC} ${${EXECUTABLE_TARGET_NAME}_HDR})
ament_target_dependencies(${EXECUTABLE_TARGET_NAME} rclcpp std_msgs controller_msgs)

install(TARGETS
  ${EXECUTABLE_TARGET_NAME}
  DESTINATION bin/${PROJECT_NAME}
)

ament_package()

