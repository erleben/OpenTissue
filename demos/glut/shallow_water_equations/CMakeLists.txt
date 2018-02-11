ADD_EXECUTABLE(shallow_water_equations src/application.cpp)

TARGET_LINK_LIBRARIES(shallow_water_equations ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS shallow_water_equations 
  RUNTIME DESTINATION  bin/demos/glut/
  )

