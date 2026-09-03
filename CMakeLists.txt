cmake_minimum_required(VERSION 3.20)
project(computer-enhance CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set to 0 to compile the per-zone profiler out entirely (only the total run time remains).
set(PROFILER "1" CACHE STRING "Enable the per-zone profiler (1) or compile it out (0)")
add_compile_definitions(PROFILER=${PROFILER})

# Fix for clangd in Nix: add implicit include directories to compile_commands.json
if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()

add_subdirectory(src/sim8086)
add_subdirectory(src/haversine)
add_subdirectory(src/profiler)

enable_testing()
add_subdirectory(src/sim8086/tests)
