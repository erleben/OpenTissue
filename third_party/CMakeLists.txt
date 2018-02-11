#-----------------------------------------------------------------------------
#
# Use the CMake Install functionality
#
INSTALL(DIRECTORY ./ DESTINATION third_party
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
#IF(WIN32)
#  FILE( GLOB_RECURSE DLL_FILES *.dll )
#  FOREACH(file ${DLL_FILES})
#    CONFIGURE_FILE( ${file}  ${LIBRARY_OUTPUT_PATH} COPYONLY )
#  ENDFOREACH( file ${DLL_FILES})
#ENDIF(WIN32)

SUBDIRS(bundled)
