ADD_EXECUTABLE(signed_distance_field_collision_detection src/application.cpp)

TARGET_LINK_LIBRARIES(signed_distance_field_collision_detection ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS signed_distance_field_collision_detection
  RUNTIME DESTINATION  bin/demos/glut/
  )

