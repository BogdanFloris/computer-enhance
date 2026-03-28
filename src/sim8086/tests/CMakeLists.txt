find_package(GTest REQUIRED)

add_executable(sim8086_tests
    decoder_test.cpp
)

target_link_libraries(sim8086_tests GTest::gtest GTest::gtest_main sim8086_lib)
target_include_directories(sim8086_tests PRIVATE ${CMAKE_SOURCE_DIR}/src/sim8086)

include(GoogleTest)
gtest_discover_tests(sim8086_tests)

