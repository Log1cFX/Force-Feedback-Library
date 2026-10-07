# Standalone Force Feedback library
# Builds as a static library with no external dependencies.

cmake_minimum_required(VERSION 3.13)
project(ffb LANGUAGES CXX C)

option(FFB_BUILD_EXAMPLES "Build the example programs" OFF)
option(FFB_BUILD_TESTS    "Build the smoke test"          OFF)
option(FFB_BUILD_C_WRAPPER "Build the C-API wrapper (ffb_c.h/cpp)" ON)

# ffb_config.h (the compile-time options) is supplied by the project that uses
# the library, not by the library. FFB_CONFIG_DIR is the directory holding it;
# examples/ffb_config.h is a template to copy.
if(NOT DEFINED FFB_CONFIG_DIR)
    set(FFB_CONFIG_DIR "" CACHE PATH "Directory containing your ffb_config.h")
endif()
if(NOT FFB_CONFIG_DIR)
    if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
        # Library built on its own (examples / tests): use the template.
        set(FFB_CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    else()
        message(FATAL_ERROR
            "ffb: FFB_CONFIG_DIR is not set. Copy examples/ffb_config.h into "
            "your project and set FFB_CONFIG_DIR to the directory that holds "
            "it before add_subdirectory().")
    endif()
endif()
get_filename_component(FFB_CONFIG_DIR "${FFB_CONFIG_DIR}" ABSOLUTE
                       BASE_DIR "${CMAKE_SOURCE_DIR}")
if(NOT EXISTS "${FFB_CONFIG_DIR}/ffb_config.h")
    message(FATAL_ERROR "ffb: no ffb_config.h found in ${FFB_CONFIG_DIR}")
endif()
message(STATUS "ffb: using ${FFB_CONFIG_DIR}/ffb_config.h")

set(FFB_CORE_SOURCES
    src/ffb_biquad.cpp
    src/ffb_calculator.cpp
    src/ffb_parser.cpp
    src/ffb_descriptor.cpp
)

# Opt-in helpers; users link them only if they include the matching headers.
set(FFB_OPTIONAL_SOURCES
    src/ffb_metrics.cpp
    src/ffb_axis_local.cpp
)

set(FFB_SOURCES ${FFB_CORE_SOURCES} ${FFB_OPTIONAL_SOURCES})
if(FFB_BUILD_C_WRAPPER)
    list(APPEND FFB_SOURCES
        src/ffb_c.cpp
        src/ffb_metrics_c.cpp      # C wrapper for the opt-in metrics helper
        src/ffb_axis_local_c.cpp   # C wrapper for the opt-in axis-local helper
    )
endif()

add_library(ffb STATIC ${FFB_SOURCES})
target_include_directories(ffb PUBLIC include "${FFB_CONFIG_DIR}")
target_compile_features(ffb PUBLIC cxx_std_11)
set_target_properties(ffb PROPERTIES CXX_EXTENSIONS OFF)

# Common compile knobs, all overridable from the parent project.
if(NOT MSVC)
    target_compile_options(ffb PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

if(FFB_BUILD_EXAMPLES)
    add_executable(ffb_example_cpp examples/minimal_cpp.cpp)
    target_link_libraries(ffb_example_cpp PRIVATE ffb)

    if(FFB_BUILD_C_WRAPPER)
        add_executable(ffb_example_c examples/minimal_c.c)
        target_link_libraries(ffb_example_c PRIVATE ffb)

        add_executable(ffb_example_c_wrappers examples/c_wrappers.c)
        target_link_libraries(ffb_example_c_wrappers PRIVATE ffb)
    endif()
endif()

if(FFB_BUILD_TESTS)
    enable_testing()
    add_executable(ffb_smoke_test tests/smoke_test.cpp)
    target_link_libraries(ffb_smoke_test PRIVATE ffb)
    add_test(NAME ffb_smoke COMMAND ffb_smoke_test)
endif()
