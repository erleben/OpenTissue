ADD_EXECUTABLE(mesh_top_down_bvh src/application.cpp)

TARGET_LINK_LIBRARIES(mesh_top_down_bvh ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS mesh_top_down_bvh
  RUNTIME DESTINATION  bin/demos/glut/
  )


