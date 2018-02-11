ADD_EXECUTABLE(t4mesh_coupling src/application.cpp)

TARGET_LINK_LIBRARIES(t4mesh_coupling ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS t4mesh_coupling 
  RUNTIME DESTINATION  bin/demos/glut/
  )


