ADD_EXECUTABLE(particle_system src/application.cpp)

TARGET_LINK_LIBRARIES(particle_system ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS particle_system 
  RUNTIME DESTINATION  bin/demos/glut/
  )

