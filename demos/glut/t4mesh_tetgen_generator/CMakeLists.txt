ADD_EXECUTABLE(t4mesh_tetgen_generator src/application.cpp)

TARGET_LINK_LIBRARIES(t4mesh_tetgen_generator ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS t4mesh_tetgen_generator
  RUNTIME DESTINATION  bin/demos/glut/
  )
