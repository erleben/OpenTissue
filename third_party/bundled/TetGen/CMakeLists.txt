ADD_LIBRARY(TetGen 
            predicates.cxx
            tetgen.cxx 
            )

#-----------------------------------------------------------------------------
# TetGen got a main-function in it, this pre-processor
# directive is used to cut that main-function declaration out of TetGen.
#
# If this is not done then every other target that tries to link with
# the tetgen static library will result in a linker error due to multiple
# defined main-functions.
SET_TARGET_PROPERTIES(TetGen PROPERTIES COMPILE_FLAGS "-D\"TETLIBRARY\"")

#-----------------------------------------------------------------------------
#
# Change the output name such that name reflect whether we have a debug
# or release version of the static libary. This is only needed on windows.
#
IF(WIN32)
  SET_TARGET_PROPERTIES(TetGen PROPERTIES DEBUG_POSTFIX "D") 
ENDIF(WIN32)

#-----------------------------------------------------------------------------
#
# Copy static lib into correct folder using a post-build event
#
IF(WIN32)
  ADD_CUSTOM_COMMAND(
    TARGET TetGen
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy ${OpenTissue_BINARY_DIR}/third_party/bundled/TetGen/debug/TetGenD.lib ${OpenTissue_SOURCE_DIR}/third_party/lib
  )
  ADD_CUSTOM_COMMAND(
    TARGET TetGen
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy ${OpenTissue_BINARY_DIR}/third_party/bundled/TetGen/release/TetGen.lib ${OpenTissue_SOURCE_DIR}/third_party/lib
  )
ELSE(WIN32)
  GET_TARGET_PROPERTY(TETGEN_LOCATION TetGen LOCATION)

  ADD_CUSTOM_COMMAND(
    TARGET TetGen
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy ${TETGEN_LOCATION} ${OpenTissue_SOURCE_DIR}/third_party/lib/
  )
ENDIF(WIN32)

MAKE_DIRECTORY("${OpenTissue_SOURCE_DIR}/third_party/include/TetGen/")
FILE( GLOB_RECURSE TETGEN_HEADER_FILES *.h )
FOREACH( file ${TETGEN_HEADER_FILES})
  ADD_CUSTOM_COMMAND(
    TARGET TetGen
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy ${file} ${OpenTissue_SOURCE_DIR}/third_party/include/TetGen/
  )
ENDFOREACH( file ${TETGEN_HEADER_FILES})

#-----------------------------------------------------------------------------
#
# Use the CMake Install functionality
#
FILE( GLOB_RECURSE TETGEN_HEADER_FILES *.h )
INSTALL(TARGETS TetGen ARCHIVE DESTINATION third_party/lib)
INSTALL( FILES ${TETGEN_HEADER_FILES} DESTINATION third_party/include/TetGen )
