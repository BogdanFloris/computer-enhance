cmake_minimum_required(VERSION 3.20)
project(computer-enhance CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Fix for clangd in Nix: add implicit include directories to compile_commands.json
if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()

add_subdirectory(src/sim8086)
add_subdirectory(src/haversine)

enable_testing()
add_subdirectory(src/sim8086/tests)
