#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "scanRegistrationLIB" for configuration "Release"
set_property(TARGET scanRegistrationLIB APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(scanRegistrationLIB PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libscanRegistrationLIB.so"
  IMPORTED_SONAME_RELEASE "libscanRegistrationLIB.so"
  )

list(APPEND _cmake_import_check_targets scanRegistrationLIB )
list(APPEND _cmake_import_check_files_for_scanRegistrationLIB "${_IMPORT_PREFIX}/lib/libscanRegistrationLIB.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
