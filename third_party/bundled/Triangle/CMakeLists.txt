ADD_LIBRARY(Triangle
            triangle.c
            )

#-----------------------------------------------------------------------------
# Triangle got a main-function in it, this pre-processor
# directive is used to cut that main-function declaration out of Triangle.
#
# If this is not done then every other target that tries to link with
# the tetgen static library will result in a linker error due to multiple
# defined main-functions.
SET(TRIANGLE_FLAGS "-D\"TRILIBRARY\"")

#-----------------------------------------------------------------------------
# Triangle library got inbuilt timers, we do not need those so we turn them
# off by defining the pre-directive NO_TIMER
#
SET(TRIANGLE_FLAGS  "${TRIANGLE_FLAGS} -D\"NO_TIMER\"")


SET_TARGET_PROPERTIES(Triangle PROPERTIES COMPILE_FLAGS ${TRIANGLE_FLAGS} )

#-----------------------------------------------------------------------------
#
# Change the output name such that name reflect whether we have a debug
# or release version of the static libary. This is only needed on windows.
#
IF(WIN32)
  SET_TARGET_PROPERTIES(Triangle PROPERTIES DEBUG_POSTFIX "D") 
ENDIF(WIN32)

#-----------------------------------------------------------------------------
#
# Copy static lib into correct folder using a post-build event
#
IF(WIN32)
  ADD_CUSTOM_COMMAND(
    TARGET Triangle
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy ${OpenTissue_BINARY_DIR}/third_party/bundled/Triangle/debug/TriangleD.lib ${OpenTissue_SOURCE_DIR}/third_party/lib/windows
  )
  ADD_CUSTOM_COMMAND(
    TARGET Triangle
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy ${OpenTissue_BINARY_DIR}/third_party/bundled/Triangle/release/Triangle.lib ${OpenTissue_SOURCE_DIR}/third_party/lib/windows
  )
ELSE(WIN32)
  GET_TARGET_PROPERTY(TRIANGLE_LOCATION Triangle LOCATION)

  ADD_CUSTOM_COMMAND(
    TARGET Triangle
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy ${TRIANGLE_LOCATION} ${OpenTissue_SOURCE_DIR}/third_party/lib/linux/
  )
ENDIF(WIN32)

MAKE_DIRECTORY("${OpenTissue_SOURCE_DIR}/third_party/include/Triangle/")
FILE( GLOB_RECURSE TRIANGLE_HEADER_FILES *.h )
FOREACH( file ${TRIANGLE_HEADER_FILES} )
  ADD_CUSTOM_COMMAND(
    TARGET Triangle
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy ${file} ${OpenTissue_SOURCE_DIR}/third_party/include/Triangle/
  )
ENDFOREACH( file ${TRIANGLE_HEADER_FILES})

#-----------------------------------------------------------------------------
#
# Use the CMake Install functionality
#
FILE( GLOB_RECURSE TRIANGLE_HEADER_FILES *.h )
IF(WIN32)
  INSTALL(TARGETS Triangle ARCHIVE DESTINATION third_party/lib/windows)
ELSE(WIN32)
  INSTALL(TARGETS Triangle ARCHIVE DESTINATION third_party/lib/linux)
ENDIF(WIN32)
INSTALL( FILES ${TRIANGLE_HEADER_FILES} DESTINATION third_party/include/Triangle )