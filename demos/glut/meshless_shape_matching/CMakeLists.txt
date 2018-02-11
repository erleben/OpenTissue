ADD_EXECUTABLE(meshless_shape_matching src/application.cpp)

TARGET_LINK_LIBRARIES(meshless_shape_matching ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS meshless_shape_matching 
  RUNTIME DESTINATION  bin/demos/glut/
  )

