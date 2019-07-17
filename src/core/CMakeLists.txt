find_package(casadi REQUIRED)
find_package(PythonLibs 2.7)

### Compile- and install-related commands.
add_subdirectory(deps)
add_subdirectory(src)

if(BUILD_TESTING)
    add_subdirectory(test)
endif()
