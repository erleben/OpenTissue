#------------------------------------------------------------------
#
# Set up a target for generating the doxygen documentation
#
#
# Find Doxygen
FIND_PACKAGE(Doxygen)

IF (DOXYGEN_FOUND)
  SET(DOXYGEN_INPUT  ${PROJECT_SOURCE_DIR}/Doxyfile )
  SET(DOXYGEN_OUTPUT ${PROJECT_SOURCE_DIR}/doc      )

  #   ADD_CUSTOM_COMMAND(
  #     OUTPUT ${DOXYGEN_OUTPUT}
  #     COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
  #     COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_INPUT}
  #     COMMAND ${CMAKE_COMMAND} -E echo "Done."
  #     WORKING_DIRECTORY ${OpenTissue_SOURCE_DIR}
  #     DEPENDS ${DOXYGEN_INPUT}
  #     )
  #
  #   ADD_CUSTOM_TARGET(doc ALL DEPENDS ${DOXYGEN_OUTPUT} )
  ADD_CUSTOM_TARGET(DOCUMENTATION
    COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_INPUT}
    COMMAND ${CMAKE_COMMAND} -E echo "Done."
    WORKING_DIRECTORY ${OpenTissue_SOURCE_DIR}
    )
  
  #-----------------------------------------------------------------------------
  #
  # Use the CMake Install functionality
  #
  INSTALL(DIRECTORY ./ DESTINATION doc
    PATTERN ".svn" EXCLUDE
    PATTERN "CMakeLists.txt" EXCLUDE
    PATTERN "debug" EXCLUDE
    PATTERN "release" EXCLUDE
    )
  
ENDIF (DOXYGEN_FOUND)


