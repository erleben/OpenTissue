ADD_EXECUTABLE(volume_fitting src/application.cpp)

TARGET_LINK_LIBRARIES(volume_fitting ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS volume_fitting
  RUNTIME DESTINATION  bin/demos/glut/
  )


