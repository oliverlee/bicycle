add_executable(bicycle_model bicycle_model.cc ${BICYCLE_SOURCE})
add_executable(kalman kalman.cc ${BICYCLE_SOURCE})
add_executable(lqr lqr.cc ${BICYCLE_SOURCE})
add_executable(lqr_kalman lqr_kalman.cc ${BICYCLE_SOURCE})
add_executable(state_space_map state_space_map.cc ${BICYCLE_SOURCE})

# All executables are built, so only copy the matrix file once.
add_custom_command(TARGET bicycle_model POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    ${BICYCLE_SOURCE_DIR}/benchmark_matrices.txt ${CMAKE_CURRENT_BINARY_DIR})
