ADD_EXECUTABLE(multibody_cows src/application.cpp)

TARGET_LINK_LIBRARIES(multibody_cows ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS multibody_cows 
  RUNTIME DESTINATION  bin/demos/glut/
  )

