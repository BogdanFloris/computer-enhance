add_executable(haversine main.cpp generator.cpp processor.cpp)
target_include_directories(haversine PRIVATE ${CMAKE_SOURCE_DIR}/src/profiler)
