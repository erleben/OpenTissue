ADD_EXECUTABLE(obb_tree src/application.cpp)

TARGET_LINK_LIBRARIES(obb_tree ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS obb_tree 
  RUNTIME DESTINATION  bin/demos/glut/
  )
