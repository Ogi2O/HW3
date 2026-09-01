#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "TinyNPY::TinyNPY" for configuration "Release"
set_property(TARGET TinyNPY::TinyNPY APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(TinyNPY::TinyNPY PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libTinyNPY.a"
  )

list(APPEND _cmake_import_check_targets TinyNPY::TinyNPY )
list(APPEND _cmake_import_check_files_for_TinyNPY::TinyNPY "${_IMPORT_PREFIX}/lib/libTinyNPY.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
