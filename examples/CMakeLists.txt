add_library(bicycle ${BICYCLE_SOURCE})

add_executable(bicycle_model bicycle_model.cc)
add_executable(bicycle_arend_model bicycle_arend_model.cc)
add_executable(bicycle_kinematic_model bicycle_kinematic_model.cc)
add_executable(bicycle_param bicycle_param.cc)

add_executable(kalman kalman.cc)
add_executable(lqr lqr.cc)
add_executable(lqr_kalman lqr_kalman.cc)
add_executable(bicycle_fbs bicycle_fbs.cc)
add_executable(full_fbs full_fbs.cc)
add_executable(udp udp.cc)
add_executable(udp_send_receive udp_send_receive.cc)
add_executable(serial serial.cc)

add_dependencies(bicycle_fbs generate_flatbuffer_headers)
add_dependencies(full_fbs generate_flatbuffer_headers)

target_link_libraries(bicycle_model bicycle)
target_link_libraries(bicycle_kinematic_model bicycle)
target_link_libraries(bicycle_arend_model bicycle)
target_link_libraries(bicycle_param bicycle)

target_link_libraries(kalman bicycle)
target_link_libraries(lqr bicycle)
target_link_libraries(lqr_kalman bicycle)
target_link_libraries(bicycle_fbs flatbuffers bicycle)
target_link_libraries(full_fbs flatbuffers bicycle)
target_link_libraries(udp bicycle)
target_link_libraries(udp_send_receive bicycle)
target_link_libraries(serial bicycle)

add_executable(bicycle_no_discretization
    bicycle_no_discretization.cc ${BICYCLE_SOURCE})
target_compile_definitions(bicycle_no_discretization
    PUBLIC BICYCLE_NO_DISCRETIZATION)
