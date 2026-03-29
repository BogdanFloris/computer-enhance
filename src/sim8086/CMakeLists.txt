add_library(sim8086_lib
    decoder.cpp
    table.cpp
    # simulator.cpp
)
target_include_directories(sim8086_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(sim8086 main.cpp)
target_link_libraries(sim8086 sim8086_lib)
target_include_directories(sim8086 PRIVATE ${CMAKE_SOURCE_DIR}/src/profiler)
