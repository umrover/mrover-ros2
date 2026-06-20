#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "dawn::webgpu_dawn" for configuration "Release"
set_property(TARGET dawn::webgpu_dawn APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(dawn::webgpu_dawn PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib64/libwebgpu_dawn.a"
  )

list(APPEND _cmake_import_check_targets dawn::webgpu_dawn )
list(APPEND _cmake_import_check_files_for_dawn::webgpu_dawn "${_IMPORT_PREFIX}/lib64/libwebgpu_dawn.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
