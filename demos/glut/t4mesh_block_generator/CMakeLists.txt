ADD_EXECUTABLE(t4mesh_block_generator src/application.cpp)

TARGET_LINK_LIBRARIES(t4mesh_block_generator ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS t4mesh_block_generator 
  RUNTIME DESTINATION  bin/demos/glut/
  )


