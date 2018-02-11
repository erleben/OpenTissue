ADD_EXECUTABLE(character_animation src/application.cpp)

TARGET_LINK_LIBRARIES(character_animation ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS character_animation
  RUNTIME DESTINATION  bin/demos/glut/
  )
