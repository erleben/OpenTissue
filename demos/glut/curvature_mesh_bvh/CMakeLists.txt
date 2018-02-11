ADD_EXECUTABLE(curvature_mesh_bvh src/application.cpp)

TARGET_LINK_LIBRARIES(curvature_mesh_bvh ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS curvature_mesh_bvh 
  RUNTIME DESTINATION  bin/demos/glut/
  )

