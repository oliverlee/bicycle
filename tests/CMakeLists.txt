file(COPY ../benchmark_matrices.txt DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

add_executable(test_state_space test_state_space.cc ${BICYCLE_SOURCE})
target_link_libraries(test_state_space gtest_main)

add_test(NAME test_state_space COMMAND test_state_space)
