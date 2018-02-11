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
# Copy static lib into correct folder using a post-build event
#
IF(WIN32)
  ADD_CUSTOM_COMMAND(
    TARGET TinyXML
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy ${OpenTissue_BINARY_DIR}/third_party/bundled/TinyXML/debug/TinyXMLD.lib ${OpenTissue_SOURCE_DIR}/third_party/lib/windows
  )
  ADD_CUSTOM_COMMAND(
    TARGET TinyXML
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy ${OpenTissue_BINARY_DIR}/third_party/bundled/TinyXML/release/TinyXML.lib ${OpenTissue_SOURCE_DIR}/third_party/lib/windows
  )
ELSE(WIN32)
  GET_TARGET_PROPERTY(TINYXML_LOCATION TinyXML LOCATION)

  ADD_CUSTOM_COMMAND(
    TARGET TinyXML
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy ${TINYXML_LOCATION} ${OpenTissue_SOURCE_DIR}/third_party/lib/linux/
  )
ENDIF(WIN32)

MAKE_DIRECTORY("${OpenTissue_SOURCE_DIR}/third_party/include/TinyXML/")
FILE( GLOB_RECURSE TINYXML_HEADER_FILES *.h )
FOREACH( file ${TINYXML_HEADER_FILES})
  ADD_CUSTOM_COMMAND(
    TARGET TinyXML
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy ${file} ${OpenTissue_SOURCE_DIR}/third_party/include/TinyXML/
  )
ENDFOREACH( file ${TINYXML_HEADER_FILES})

#-----------------------------------------------------------------------------
#
# Use the CMake Install functionality
#
FILE( GLOB_RECURSE TINYXML_HEADER_FILES *.h )
IF(WIN32)
  INSTALL(TARGETS TinyXML ARCHIVE DESTINATION third_party/lib/windows)
ELSE(WIN32)
  INSTALL(TARGETS TinyXML ARCHIVE DESTINATION third_party/lib/linux)
ENDIF(WIN32)
INSTALL( FILES ${TINYXML_HEADER_FILES} DESTINATION third_party/include/TinyXML )