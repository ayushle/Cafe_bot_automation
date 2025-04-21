#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "cafe_bot::cafe_bot__rosidl_generator_py" for configuration ""
set_property(TARGET cafe_bot::cafe_bot__rosidl_generator_py APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(cafe_bot::cafe_bot__rosidl_generator_py PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libcafe_bot__rosidl_generator_py.so"
  IMPORTED_SONAME_NOCONFIG "libcafe_bot__rosidl_generator_py.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS cafe_bot::cafe_bot__rosidl_generator_py )
list(APPEND _IMPORT_CHECK_FILES_FOR_cafe_bot::cafe_bot__rosidl_generator_py "${_IMPORT_PREFIX}/lib/libcafe_bot__rosidl_generator_py.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
