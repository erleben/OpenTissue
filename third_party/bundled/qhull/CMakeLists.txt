#------------------------------------------------------
#
# QHull is distributed with Gentoo-Portage, so we only need to built it ourself
# on windows.
#
IF(WIN32)

  ADD_LIBRARY(qhull 
    geom.c   
    global.c  
    mem.c    
    poly.c   
    qhull.c  
    stat.c
    geom2.c  
    io.c      
    merge.c  
    poly2.c  
    qset.c   
    user.c
    )

  #-----------------------------------------------------------------------------
  #
  # Change the output name such that name reflect whether we have a debug
  # or release version of the static libary. This is only needed on windows.
  #
  IF(WIN32)
    SET_TARGET_PROPERTIES(qhull PROPERTIES DEBUG_POSTFIX "D") 
  ENDIF(WIN32)

  #-----------------------------------------------------------------------------
  #
  # Copy static lib into correct folder using a post-build event
  #
  ADD_CUSTOM_COMMAND(
    TARGET qhull
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy ${OpenTissue_BINARY_DIR}/third_party/bundled/qhull/debug/qhullD.lib ${OpenTissue_SOURCE_DIR}/third_party/lib/windows
    )
  ADD_CUSTOM_COMMAND(
    TARGET qhull
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy ${OpenTissue_BINARY_DIR}/third_party/bundled/qhull/release/qhull.lib ${OpenTissue_SOURCE_DIR}/third_party/lib/windows
    )

  MAKE_DIRECTORY("${OpenTissue_SOURCE_DIR}/third_party/include/windows/qhull/")
  FILE( GLOB_RECURSE QHULL_HEADER_FILES *.h )
  FOREACH( file ${QHULL_HEADER_FILES})
    ADD_CUSTOM_COMMAND(
      TARGET qhull
      POST_BUILD
      COMMAND ${CMAKE_COMMAND}
      ARGS -E copy ${file} ${OpenTissue_SOURCE_DIR}/third_party/include/windows/qhull/
      )
  ENDFOREACH( file ${QHULL_HEADER_FILES})

  #-----------------------------------------------------------------------------
  #
  # Use the CMake Install functionality
  #
  FILE( GLOB_RECURSE QHULL_HEADER_FILES *.h )
  INSTALL(TARGETS qhull ARCHIVE DESTINATION third_party/lib/windows)
  INSTALL( FILES ${QHULL_HEADER_FILES} DESTINATION third_party/include/windows/qhull )
  
ENDIF(WIN32)
