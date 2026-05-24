cmake_minimum_required(VERSION 3.15...3.28)
project(SwarmCore VERSION 1.0 LANGUAGES CXX)

# Set C++20 Standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Python and PyBind11
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# This executes python to find where pip installed pybind11
execute_process(
    COMMAND "${Python3_EXECUTABLE}" -m pybind11 --includes
    OUTPUT_VARIABLE PYBIND11_INCLUDES
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
# Clean up the output string to use as include directories
string(REGEX REPLACE "-I" "" PYBIND11_INCLUDES "${PYBIND11_INCLUDES}")
separate_arguments(PYBIND11_INCLUDES)

# Create the native Python extension (the .pyd file)
Python3_add_library(fast_spatial_engine MODULE src/engine.cpp)

# Link PyBind11 headers
target_include_directories(fast_spatial_engine PRIVATE ${PYBIND11_INCLUDES})

# Apply maximum speed optimizations for the compiler
if(MSVC)
    target_compile_options(fast_spatial_engine PRIVATE /O2 /Oi /GL /fp:fast)
    set_target_properties(fast_spatial_engine PROPERTIES LINK_FLAGS "/LTCG")
endif()