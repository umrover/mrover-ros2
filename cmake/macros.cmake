find_program(CLANG_TIDY_PROGRAM clang-tidy-18)

# General macro to enable properties for MRover targets (and not 3rd party).
macro(mrover_target name)
    if (MROVER_CLANG_TIDY)
        if (NOT CLANG_TIDY_PROGRAM)
            message(FATAL_ERROR "Clang tidy requested but executable not found")
        endif ()
        message(STATUS "Enabling static analysis for target ${name}")
        set_target_properties(${name} PROPERTIES CXX_CLANG_TIDY "${CLANG_TIDY_PROGRAM}")
    endif ()
endmacro()

macro(mrover_add_header_only_library name includes)
    add_library(${name} INTERFACE)
    mrover_target(${name})
    target_include_directories(${name} INTERFACE ${includes})
endmacro()

macro(mrover_add_library name sources includes)
    file(GLOB_RECURSE LIBRARY_SOURCES CONFIGURE_DEPENDS ${sources})
    add_library(${name} ${ARGV3} ${LIBRARY_SOURCES})
    mrover_target(${name})
    target_include_directories(${name} PUBLIC ${includes})
endmacro()

macro(mrover_add_component name sources includes)
    file(GLOB_RECURSE LIBRARY_SOURCES CONFIGURE_DEPENDS ${sources})
    add_library(${name} SHARED ${ARGV3} ${LIBRARY_SOURCES})
    mrover_target(${name})
    target_compile_definitions(${name} PRIVATE "COMPOSITION_BUILDING_DLL")
endmacro()  

macro(mrover_add_node name sources)
    file(GLOB_RECURSE NODE_SOURCES CONFIGURE_DEPENDS ${sources})
    add_executable(${name} ${NODE_SOURCES})
    mrover_target(${name})
    if (ARGV3)
        target_precompile_headers(${name} PRIVATE ${ARGV3})
    endif ()
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
    target_link_libraries(${name} ${cpp_typesupport_target})
    install(
            TARGETS ${name}
            DESTINATION lib/${PROJECT_NAME}
    )
endmacro()
