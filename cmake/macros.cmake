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
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
    target_link_libraries(${name} INTERFACE ${cpp_typesupport_target})
endmacro()

macro(mrover_add_library name sources includes)
    file(GLOB_RECURSE LIBRARY_SOURCES CONFIGURE_DEPENDS ${sources})
    list(FILTER LIBRARY_SOURCES EXCLUDE REGEX ".*/main.*\.cpp")

    ### VARIABLES FOR COMPONENTS ###

    # there must be at least one source
    message("Sources ${LIBRARY_SOURCES}")
    list(GET LIBRARY_SOURCES 0 ${name}_first_source)
    message("First source ${${name}_first_source}")
    # gets the parent directory of this executable
    cmake_path(GET ${name}_first_source PARENT_PATH ${name}_main_dir)
    message("${name}_main_dir source directory ${${name}_main_dir}")

    ### VARIABLES FOR COMPONENTS ###

    add_library(${name} ${ARGV3} ${LIBRARY_SOURCES})
    mrover_target(${name})
    target_include_directories(${name} PUBLIC ${includes})
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
    target_link_libraries(${name} ${cpp_typesupport_target})
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

macro(mrover_add_component name sources includes)
    # create composition library
    mrover_add_library(${name}_component ${sources} ${includes} SHARED)
    # creates the variable for the main file
    rosidl_target_interfaces(${name}_component ${PROJECT_NAME} "rosidl_typesupport_cpp")
    foreach(node ${ARGN})
        rclcpp_components_register_nodes(${name}_component "mrover::${node}")
    endforeach()
    target_compile_definitions(${name}_component PRIVATE "COMPOSITION_BUILDING_DLL")
    install(CODE "execute_process( \
    COMMAND ${CMAKE_COMMAND} -E create_symlink \
    ${CMAKE_CURRENT_LIST_DIR}/../../build/${PROJECT_NAME}/lib${name}_component.so \
    ${CMAKE_CURRENT_LIST_DIR}/../../install/${PROJECT_NAME}/lib/lib${name}_component.so \
    )"
)
endmacro()  

macro(mrover_executable_from_component name library)
    if(EXISTS "${${library}_main_dir}/main.cpp")
        message("${${library}_main_dir}/main.cpp exists...")
        mrover_add_node(${name} ${${library}_main_dir}/main.cpp)
    else()
        message("${${library}_main_dir}/main.cpp does not exist...")
        if(EXISTS "${${library}_main_dir}/main.${name}.cpp")
            message("${${library}_main_dir}/main.${name}.cpp exists...")
            mrover_add_node(${name} ${${library}_main_dir}/main.${name}.cpp)
        else()
            message(FATAL_ERROR "${${library}_main_dir}/main.${name}.cpp does not exist...")
        endif()
    endif()
    target_link_libraries(${name} ${library})
endmacro()

macro(mrover_link_component name)
    target_link_libraries(${name}_component ${ARGN})
endmacro()

macro(mrover_ament_component name)
    ament_target_dependencies(${name}_component ${ARGN})
endmacro()
