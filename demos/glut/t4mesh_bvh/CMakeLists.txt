ADD_EXECUTABLE(t4mesh_bvh src/application.cpp)

TARGET_LINK_LIBRARIES(t4mesh_bvh ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS t4mesh_bvh 
  RUNTIME DESTINATION  bin/demos/glut/
  )

