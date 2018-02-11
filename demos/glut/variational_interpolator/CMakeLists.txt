ADD_EXECUTABLE(variational_interpolator src/application.cpp)

TARGET_LINK_LIBRARIES(variational_interpolator ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS variational_interpolator
  RUNTIME DESTINATION  bin/demos/glut/
  )


