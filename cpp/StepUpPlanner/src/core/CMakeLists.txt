find_package(casadi REQUIRED)

### Compile- and install-related commands.
add_subdirectory(src)

# Add integration tests (unit tests for each library should be in each sublibrary directory).
if(BUILD_TESTING)
    add_subdirectory(test)
endif()
