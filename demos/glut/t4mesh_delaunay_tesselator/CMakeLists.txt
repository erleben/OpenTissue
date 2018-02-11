ADD_EXECUTABLE(t4mesh_delaunay_tesselator src/application.cpp)

TARGET_LINK_LIBRARIES(t4mesh_delaunay_tesselator ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS t4mesh_delaunay_tesselator
  RUNTIME DESTINATION  bin/demos/glut/
  )

