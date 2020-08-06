#-----------------------------------------------------------------------------
#
# Use the CMake Install functionality
#
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION demos/data
  COMPONENT Demos
  PATTERN ".svn" EXCLUDE
  PATTERN "CMakeLists.txt" EXCLUDE
  PATTERN "debug" EXCLUDE
  PATTERN "release" EXCLUDE
  )
