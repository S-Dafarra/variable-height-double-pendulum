find_package(casadi REQUIRED)

### Compile- and install-related commands.
add_subdirectory(src)

if(BUILD_TESTING)
    add_subdirectory(test)
endif()
