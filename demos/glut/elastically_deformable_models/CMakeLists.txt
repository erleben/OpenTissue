ADD_EXECUTABLE(elastically_deformable_models src/application.cpp)

TARGET_LINK_LIBRARIES(elastically_deformable_models ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS elastically_deformable_models
  RUNTIME DESTINATION  bin/demos/glut/
  )
