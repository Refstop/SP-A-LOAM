#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "SP_A_LOAM" for configuration "Release"
set_property(TARGET SP_A_LOAM APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(SP_A_LOAM PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libSP_A_LOAM.so"
  IMPORTED_SONAME_RELEASE "libSP_A_LOAM.so"
  )

list(APPEND _cmake_import_check_targets SP_A_LOAM )
list(APPEND _cmake_import_check_files_for_SP_A_LOAM "${_IMPORT_PREFIX}/lib/libSP_A_LOAM.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
