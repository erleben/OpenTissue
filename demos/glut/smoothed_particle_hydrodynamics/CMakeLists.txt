ADD_EXECUTABLE(smoothed_particle_hydrodynamics src/application.cpp)

TARGET_LINK_LIBRARIES(smoothed_particle_hydrodynamics ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS smoothed_particle_hydrodynamics
  RUNTIME DESTINATION  bin/demos/glut/
  )
