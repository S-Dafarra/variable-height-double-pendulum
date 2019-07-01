
set(LIBRARY_TARGET_NAME ${PROJECT_NAME})

# List of CPP (source) library files.
set(${LIBRARY_TARGET_NAME}_SRC
        src/Control.cpp
        src/CostWeights.cpp
        src/Phase.cpp
        src/References.cpp
        src/Settings.cpp
        src/Solver.cpp
        src/State.cpp
        src/Step.cpp
)

# List of HPP (header) library files.
set(${LIBRARY_TARGET_NAME}_HDR
        include/StepUpPlanner/Control.h
        include/StepUpPlanner/CostWeights.h
        include/StepUpPlanner/Phase.h
        include/StepUpPlanner/PhaseType.h
        include/StepUpPlanner/References.h
        include/StepUpPlanner/Settings.h
        include/StepUpPlanner/SideDependentObject.h
        include/StepUpPlanner/Solver.h
        include/StepUpPlanner/State.h
        include/StepUpPlanner/Step.h
)

add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_SRC} ${${LIBRARY_TARGET_NAME}_HDR})

add_library(${PROJECT_NAME}::${LIBRARY_TARGET_NAME} ALIAS ${LIBRARY_TARGET_NAME})

set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES VERSION       ${${PROJECT_NAME}_VERSION}
                                                        PUBLIC_HEADER "${${LIBRARY_TARGET_NAME}_HDR}")

target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                         "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC casadi)
# If you used find_package() you need to use target_include_directories() and/or
# target_link_libraries(). As explained previously, depending on the imported
# objects, you may need to call either or both:
# - with imported variable:
#   target_include_directories(${LIBRARY_TARGET_NAME} ${FooPackage_INCLUDE_DIRS})
#   target_link_libraries(${LIBRARY_TARGET_NAME} ${FooPackage_LIBRARIES})
# - with imported target:
#   target_link_libraries(${LIBRARY_TARGET_NAME} FooPackage_LIBRARIES::FooPackage_LIBRARIES)

# Specify installation targets, typology and destination folders.
install(TARGETS ${LIBRARY_TARGET_NAME}
        EXPORT  ${PROJECT_NAME}
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT shlib
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT lib
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                            COMPONENT bin
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}" COMPONENT dev)


message(STATUS "Created target ${LIBRARY_TARGET_NAME} for export ${PROJECT_NAME}.")
