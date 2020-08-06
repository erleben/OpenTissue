ADD_LIBRARY(TinyXML
            # tinystr.cpp   # Not used, since we want to use std::string version of TinyXML
            tinyxml.cpp
            tinyxmlerror.cpp
            tinyxmlparser.cpp
            )

#-----------------------------------------------------------------------------
# TinyXML can work both with stl strings and char*. The functionality is
# controlled by defining the pre-processor directive TIXML_USE_STL or not.
#
# Note that every target that links against TinyXML also need to have
# the TIXML_USE_STL flag defined!
SET(TINYXML_FLAGS  "-D\"TIXML_USE_STL\"")

SET_TARGET_PROPERTIES(TinyXML PROPERTIES COMPILE_FLAGS ${TINYXML_FLAGS})

#-----------------------------------------------------------------------------
#
# Change the output name such that name reflect whether we have a debug
# or release version of the static libary. This is only needed on windows.
#
IF(WIN32)
  SET_TARGET_PROPERTIES(TinyXML PROPERTIES DEBUG_POSTFIX "D")
ENDIF(WIN32)

#-----------------------------------------------------------------------------
#
# Use the CMake Install functionality
#
FILE( GLOB_RECURSE TINYXML_HEADER_FILES *.h )
INSTALL(TARGETS TinyXML ARCHIVE DESTINATION third_party/lib)
INSTALL( FILES ${TINYXML_HEADER_FILES} DESTINATION third_party/include/TinyXML )
