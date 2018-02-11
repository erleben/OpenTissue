ADD_EXECUTABLE(damped_wave_equations src/application.cpp)

TARGET_LINK_LIBRARIES(damped_wave_equations ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS damped_wave_equations
  RUNTIME DESTINATION  bin/demos/glut/
  )
