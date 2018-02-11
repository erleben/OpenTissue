#-----------------------------------------------------------------------------
#
# Create a header only target! This is really not supported in CMake, because
# a target needs to be a compilation unit, and a compilation unit needs source
# files, yrk! The stuff below is merely a workaround to fool CMake.
#
FILE( GLOB_RECURSE OPENTISSUE_HEADER_FILES *.h )

#
# MSVC specific hack!
# This works fine and we need to keep a vcproj file around and up to date all the time.
# Further it only adds support for MSVC and no other platforms:-(
#
# This is why we perfer the Compiler independent way below.
#
#INCLUDE_EXTERNAL_MSPROJECT(OpenTissue ${OpenTissue_SOURCE_DIR}/OpenTissue/OpenTissue.vcproj )

#
# Compiler Independent Way!
# We cheat CMake to create a compilation unit without any source files. We
# create a dummy cpp-file and then setup a static library target containing
# this fake cpp file.
#
SET(FAKE_CPP_FILE   "${OpenTissue_BINARY_DIR}/OpenTissue/header_only.cpp" )
# The newline is added intentionally, because linux-gcc complains if it is'nt there:
FILE(WRITE ${FAKE_CPP_FILE} "/* This file was created to cheat CMake to create a header-only compilation unit*/
")

#
# Force CMake to reflect the folder structure of our Header only library in 
# the binary/build tree. This do not seem to work in an optimal way yet!!!
#
#INCLUDE(${PROJECT_SOURCE_DIR}/GenerateProject.cmake)
#GenerateProject ( ${OpenTissue_SOURCE_DIR}/OpenTissue/ "${OPENTISSUE_HEADER_FILES}" )

ADD_LIBRARY(OpenTissue EXCLUDE_FROM_ALL ${OPENTISSUE_HEADER_FILES} ${FAKE_CPP_FILE} )

#-----------------------------------------------------------------------------
#
# Use the CMake Install functionality
#
#INSTALL(TARGETS OpenTissue ARCHIVE DESTINATION lib)
#INSTALL( FILES ${OPENTISSUE_HEADER_FILES} DESTINATION include )


INSTALL(DIRECTORY ./ DESTINATION OpenTissue
  PATTERN ".svn" EXCLUDE
  PATTERN "CMakeLists.txt" EXCLUDE
  PATTERN "*.h"
  PATTERN "debug" EXCLUDE
  PATTERN "release" EXCLUDE
  )
