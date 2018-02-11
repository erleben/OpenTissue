ADD_EXECUTABLE(mesh_clean_up src/application.cpp)

TARGET_LINK_LIBRARIES(mesh_clean_up ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS mesh_clean_up
  RUNTIME DESTINATION  bin/demos/glut/
  )

