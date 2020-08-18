#-----------------------------------------------------------------------------
#
# Use the CMake Install functionality
#
install(DIRECTORY ./ DESTINATION third_party
  PATTERN ".svn" EXCLUDE
  PATTERN "CMakeLists.txt" EXCLUDE
  PATTERN "debug" EXCLUDE
  PATTERN "release" EXCLUDE
  PATTERN "bundled" EXCLUDE
  )

#-----------------------------------------------------------------------------
#
# Deafult overriding on windows platforms, copy all dll's into the same folder as exe-files
#
#if(WIN32)
#  FILE( GLOB_RECURSE DLL_FILES *.dll )
#  foreach(file ${DLL_FILES})
#    configure_file( ${file}  ${LIBRARY_OUTPUT_PATH} COPYONLY )
#  endforeach( file ${DLL_FILES})
#endif(WIN32)

add_subdirectory(bundled)
