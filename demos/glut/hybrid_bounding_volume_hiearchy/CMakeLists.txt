ADD_EXECUTABLE(hybrid_bounding_volume_hierarchy src/application.cpp)

TARGET_LINK_LIBRARIES(hybrid_bounding_volume_hierarchy ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS hybrid_bounding_volume_hierarchy
  RUNTIME DESTINATION  bin/demos/glut/
  )
