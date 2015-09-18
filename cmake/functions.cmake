function(FLATBUFFERS_GENERATE_C_HEADERS Name)
    set(FLATC_OUTPUTS)
    foreach(FILE ${ARGN})
        get_filename_component(FLATC_OUTPUT ${FILE} NAME_WE)
        get_filename_component(SCHEMA_DIRECTORY ${FILE} DIRECTORY)
        set(FLATC_OUTPUT
          "${CMAKE_CURRENT_BINARY_DIR}/${FLATC_OUTPUT}_generated.h")
        list(APPEND FLATC_OUTPUTS ${FLATC_OUTPUT})

        add_custom_command(OUTPUT ${FLATC_OUTPUT}
          COMMAND flatc
          ARGS -c -o "${CMAKE_CURRENT_BINARY_DIR}/" ${FILE}
          COMMENT "Building C++ header for ${FILE}"
          WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    endforeach()
    set(${Name}_OUTPUTS ${FLATC_OUTPUTS} PARENT_SCOPE)
endfunction()
