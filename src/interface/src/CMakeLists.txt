
add_library(${PROJECT_NAME}_responderLibrary include/Responder.h src/Responder.cpp)

#TODO this is probably not the ROS way to link a library

target_link_libraries(${PROJECT_NAME}_responderLibrary step_up_planner::core)

target_include_directories(${PROJECT_NAME}_responderLibrary PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                            "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

ament_target_dependencies(${PROJECT_NAME}_responderLibrary rclcpp controller_msgs)


add_executable(${PROJECT_NAME}_responder src/ResponderMain.cpp)

target_link_libraries(${PROJECT_NAME}_responder ${PROJECT_NAME}_responderLibrary)

install(TARGETS
  ${PROJECT_NAME}_responder
  DESTINATION bin/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_responderLibrary
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
