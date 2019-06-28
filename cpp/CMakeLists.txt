cmake_minimum_required(VERSION 3.5)

project(StepUpPlanner
        LANGUAGES CXX
        VERSION 0.0.0.0)

include(GNUInstallDirs)

# Enable C++14
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Control where libraries and executables are placed during the build.
# With the following settings executables are placed in <the top level of the
# build tree>/bin and libraries/archives in <top level of the build tree>/lib.
# This is particularly useful to run ctests on libraries built on Windows
# machines: tests, which are executables, are placed in the same folders of
# dlls, which are treated as executables as well, so that they can properly
# find the libraries to run. This is a because of missing RPATH on Windows.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

# To build shared libraries in Windows, we set CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS to TRUE.
# See https://cmake.org/cmake/help/v3.4/variable/CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS.html
# See https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Under MSVC, we set CMAKE_DEBUG_POSTFIX to "d" to add a trailing "d" to library
# built in debug mode. In this Windows user can compile, build and install the
# library in both Release and Debug configuration avoiding naming clashes in the
# installation directories.
if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif()

# Build position independent code.
# Position Independent Code (PIC) is commonly used for shared libraries so that
# the same shared library code can be loaded in each program address space in a
# location where it will not overlap with any other uses of such memory.
# In particular, this option avoids problems occurring when a process wants to
# load more than one shared library at the same virtual address.
# Since shared libraries cannot predict where other shared libraries could be
# loaded, this is an unavoidable problem with the traditional shared library
# concept.
# Generating position-independent code is often the default behavior for most
# modern compilers.
# Moreover linking a static library that is not built with PIC from a shared
# library will fail on some compiler/architecture combinations.
# Further details on PIC can be found here:
# https://eli.thegreenplace.net/2011/11/03/position-independent-code-pic-in-shared-libraries/
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Disable C and C++ compiler extensions.
# C/CXX_EXTENSIONS are ON by default to allow the compilers to use extended
# variants of the C/CXX language.
# However, this could expose cross-platform bugs in user code or in the headers
# of third-party dependencies and thus it is strongly suggested to turn
# extensions off.
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

### Options
# Shared/Dynamic or Static library?
option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)

# Build test related commands?
option(BUILD_TESTING "Create tests using CMake" OFF)
if(BUILD_TESTING)
    enable_testing()
endif()

# Enable RPATH support for installed binaries and libraries
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                          USE_LINK_PATH)

# Encourage user to specify a build type (e.g. Release, Debug, etc.), otherwise set it to Release.
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
endif()

find_package(casadi REQUIRED)

### Compile- and install-related commands.
add_subdirectory(src)

include(InstallBasicPackageFiles)
install_basic_package_files(${PROJECT_NAME}
                            VERSION ${${PROJECT_NAME}_VERSION}
                            COMPATIBILITY AnyNewerVersion
                            VARS_PREFIX ${PROJECT_NAME}
                            NO_CHECK_REQUIRED_COMPONENTS_MACRO)


# Add the uninstall target
include(AddUninstallTarget)

# Add integration tests (unit tests for each library should be in each sublibrary directory).
if(BUILD_TESTING)
#    add_subdirectory(test)
endif()

