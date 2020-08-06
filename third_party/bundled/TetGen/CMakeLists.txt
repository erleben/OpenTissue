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
# Use the CMake Install functionality
#
FILE( GLOB_RECURSE TETGEN_HEADER_FILES *.h )
INSTALL(TARGETS TetGen ARCHIVE DESTINATION third_party/lib)
INSTALL( FILES ${TETGEN_HEADER_FILES} DESTINATION third_party/include/TetGen )
