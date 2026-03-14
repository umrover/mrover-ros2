cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)
# STM32CubeMX generated symbols (macros)
set(MX_Defines_Syms
    USE_HAL_DRIVER
    STM32G431xx
    $<$<CONFIG:Debug>:DEBUG>
)

IF(NOT DEFINED MX_SRC_DIR)
    MESSAGE(FATAL_ERROR "ERROR: Cube MX Source Directory Must Be Defined")
ENDIF()

IF(NOT DEFINED MX_INC_DIR)
    MESSAGE(FATAL_ERROR "ERROR: Cube MX Source Directory Must Be Defined")
ENDIF()

IF(NOT DEFINED MX_STARTUP_S)
    MESSAGE(FATAL_ERROR "ERROR: Cube MX Source Directory Must Be Defined")
ENDIF()


# STM32CubeMX generated include paths
set(MX_Include_Dirs
    ${MX_INC_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32CubeG4/Drivers/CMSIS/Device/ST/STM32G4xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32CubeG4/Drivers/CMSIS/Include
)

# STM32CubeMX generated application sources
set(MX_Application_Src
    ${MX_SRC_DIR}/main.c
    ${MX_SRC_DIR}/stm32g4xx_it.c
    ${MX_SRC_DIR}/stm32g4xx_hal_msp.c
    ${MX_SRC_DIR}/sysmem.c
    ${MX_SRC_DIR}/syscalls.c
    ${MX_STARTUP_S}
)

# STM32 HAL/LL Drivers
file(GLOB_RECURSE DRIVER_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/*.c"
)
list(FILTER DRIVER_SOURCES EXCLUDE REGEX ".*_template\\.c$")
set(STM32_Drivers_Src
    ${MX_SRC_DIR}/system_stm32g4xx.c
    ${DRIVER_SOURCES}
)

# Drivers Midllewares



# Link directories setup
set(MX_LINK_DIRS

)
# Project static libraries
set(MX_LINK_LIBS
    STM32_Drivers
    ${TOOLCHAIN_LINK_LIBRARIES}
)

# Interface library for includes and symbols
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

# Create STM32_Drivers static library
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)


# Add STM32CubeMX generated application sources to the project
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Application_Src})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${MX_LINK_DIRS})

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME} ${MX_LINK_LIBS})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()

