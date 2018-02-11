ADD_EXECUTABLE(vertex_mean_curvature src/application.cpp)

TARGET_LINK_LIBRARIES(vertex_mean_curvature ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS vertex_mean_curvature 
  RUNTIME DESTINATION  bin/demos/glut/
  )

