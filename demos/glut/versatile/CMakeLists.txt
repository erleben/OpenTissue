ADD_EXECUTABLE(versatile src/application.cpp)

TARGET_LINK_LIBRARIES(versatile ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS versatile 
  RUNTIME DESTINATION  bin/demos/glut/
  )



