function(FLATBUFFERS_GENERATE_C_HEADERS Name)
    set(FLATC_OUTPUTS)
    foreach(FILE ${ARGN})
        get_filename_component(FLATC_OUTPUT ${FILE} NAME_WE)
        get_filename_component(SCHEMA_DIRECTORY ${FILE} DIRECTORY)
        set(FLATC_OUTPUT "${PROJECT_BINARY_DIR}/generated/${FLATC_OUTPUT}_generated.h")
        list(APPEND FLATC_OUTPUTS ${FLATC_OUTPUT})

        add_custom_command(OUTPUT ${FLATC_OUTPUT}
          COMMAND flatc
          ARGS -c -o "${PROJECT_BINARY_DIR}/generated/" ${FILE}
          COMMENT "Building C++ header for ${FILE}"
          WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    endforeach()
    set(${Name}_OUTPUTS ${FLATC_OUTPUTS} PARENT_SCOPE)
endfunction()

function(FLATBUFFERS_GENERATE_BINARY_SCHEMA Name)
    set(FLATC_OUTPUTS)
    foreach(FILE ${ARGN})
        get_filename_component(FILE_WE ${FILE} NAME_WE)
        get_filename_component(SCHEMA_DIRECTORY ${FILE} DIRECTORY)
        set(FLATC_OUTPUT "${BINARY_PROJECT_DIR}/generated/${FILE_WE}.bfbs")
        list(APPEND FLATC_OUTPUTS ${FLATC_OUTPUT})

        add_custom_command(OUTPUT ${FLATC_OUTPUT}
          COMMAND flatc
          ARGS -b --schema -o "${PROJECT_BINARY_DIR}/generated/" ${FILE}
          COMMENT "Generating binary schema ${FILE_WE}.bfbs"
          WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    endforeach()
    set(${Name}_OUTPUTS ${FLATC_OUTPUTS} PARENT_SCOPE)
endfunction()
