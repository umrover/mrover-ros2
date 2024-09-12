macro(mrover_add_header_only_library name includes)
    add_library(${name} INTERFACE)
    target_include_directories(${name} INTERFACE ${includes})
endmacro()

macro(mrover_add_library name sources includes)
    file(GLOB_RECURSE LIBRARY_SOURCES CONFIGURE_DEPENDS ${sources})
    add_library(${name} ${ARGV3} ${LIBRARY_SOURCES})
    target_include_directories(${name} PUBLIC ${includes})
endmacro()

macro(mrover_add_component name sources includes)
    file(GLOB_RECURSE LIBRARY_SOURCES CONFIGURE_DEPENDS ${sources})
    add_library(${name} SHARED ${ARGV3} ${LIBRARY_SOURCES})
    target_compile_definitions(${name} PRIVATE "COMPOSITION_BUILDING_DLL")

endmacro()

macro(mrover_add_node name sources)
    file(GLOB_RECURSE NODE_SOURCES CONFIGURE_DEPENDS ${sources})
    add_executable(${name} ${NODE_SOURCES})
    if (ARGV3)
        target_precompile_headers(${name} PRIVATE ${ARGV3})
    endif ()
    rosidl_target_interfaces(${name} ${PROJECT_NAME} rosidl_typesupport_cpp)
    install(
            TARGETS ${name}
            DESTINATION lib/${PROJECT_NAME}
    )
endmacro()
