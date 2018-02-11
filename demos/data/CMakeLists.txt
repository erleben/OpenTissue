#-----------------------------------------------------------------------------
#
# Use the CMake Install functionality
#
INSTALL(DIRECTORY ./ DESTINATION demos/data
  PATTERN ".svn" EXCLUDE
  PATTERN "CMakeLists.txt" EXCLUDE
  PATTERN "debug" EXCLUDE
  PATTERN "release" EXCLUDE
  )
