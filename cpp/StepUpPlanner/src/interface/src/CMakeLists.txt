
add_executable(${PROJECT_NAME}_requester include/Requester.h src/Requester.cpp)

target_include_directories(${PROJECT_NAME}_requester PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                            "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

ament_target_dependencies(${PROJECT_NAME}_requester rclcpp controller_msgs)

add_executable(${PROJECT_NAME}_responder include/Responder.h src/Responder.cpp)

target_link_libraries(${PROJECT_NAME}_responder step_up_planner::core)

target_include_directories(${PROJECT_NAME}_responder PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                            "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

ament_target_dependencies(${PROJECT_NAME}_responder rclcpp controller_msgs)

install(TARGETS
  ${PROJECT_NAME}_requester
  ${PROJECT_NAME}_responder
  DESTINATION bin/${PROJECT_NAME}
)

ament_package()

