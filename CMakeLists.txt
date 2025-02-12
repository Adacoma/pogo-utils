cmake_minimum_required(VERSION 3.16)
project(pogo-utils VERSION 1.0.0 LANGUAGES C CXX)
include(GNUInstallDirs)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules" ${CMAKE_MODULE_PATH})

# Set C++ and C standards
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)


# Collect source files
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/*.c"
)
list(FILTER SOURCES EXCLUDE REGEX "/(pogobot-sdk|examples)/.*$")

# Create library target
add_library(${PROJECT_NAME} ${SOURCES})

# Include directories
target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions(${PROJECT_NAME}
    PUBLIC
        DATA_DIR="${CMAKE_INSTALL_FULL_DATADIR}/${PROJECT_NAME}"
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    PUBLIC
)

# Set compile options
#target_compile_options(${PROJECT_NAME}
#    PRIVATE
#        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wunused-parameter -O3 -msse4.1 -mpclmul -march=native>
#        $<$<CXX_COMPILER_ID:MSVC>:/W4 /O2 /arch:AVX2>
#)

#target_compile_options(${PROJECT_NAME}
#    PRIVATE
#        $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra -Wunused-parameter -O3 -msse4.1 -mpclmul -march=native>
#        $<$<CXX_COMPILER_ID:Clang>:-Wall -Wextra -Wunused-parameter -O3 -msse4.1 -mpclmul -march=native>
#        $<$<CXX_COMPILER_ID:MSVC>:/W4 /O2 /arch:AVX2>
#)

target_compile_options(${PROJECT_NAME}
    PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:
            $<$<CONFIG:Release>:-Wall -Wextra -Wunused-parameter -Os -DNDEBUG>
            $<$<CONFIG:Debug>:-Wall -Wextra -Wunused-parameter -g -Og -DDEBUG>
        >
)

# Install rules
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}  # Shared libraries (.so, .dylib)
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}  # Static libraries (.a)
    #RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}  # Executables (if any)
)

# Install header files
install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/pogo-utils/  # Path to the header files
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}  # Install location
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"  # Only install .h and .hpp files
)

# Generate and install CMake config files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${PROJECT_NAME}ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

###### BUILD EXAMPLES ######
# Find all subdirectories in examples
file(GLOB EXAMPLE_DIRS RELATIVE ${CMAKE_SOURCE_DIR}/examples ${CMAKE_SOURCE_DIR}/examples/*)

# Create a custom target that will build all examples
add_custom_target(examples #ALL
#    DEPENDS ${PROJECT_NAME}
    COMMENT "Building all examples"
)

# For each example directory
foreach(EXAMPLE ${EXAMPLE_DIRS})
    if(IS_DIRECTORY ${CMAKE_SOURCE_DIR}/examples/${EXAMPLE})
        # Create a custom target for this specific example
        add_custom_target(example_${EXAMPLE}
            COMMAND ${CMAKE_COMMAND} -E echo "Building example: ${EXAMPLE}"
            COMMAND make clean
            COMMAND make sim
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/examples/${EXAMPLE}
            DEPENDS ${PROJECT_NAME}
            COMMENT "Building example ${EXAMPLE}"
        )
        
        # Add this example target as a dependency of the main examples target
        add_dependencies(examples example_${EXAMPLE})
    endif()
endforeach()

