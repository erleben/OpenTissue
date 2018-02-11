ADD_EXECUTABLE(ellipsoid_decomposition src/application.cpp)

TARGET_LINK_LIBRARIES(ellipsoid_decomposition ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS ellipsoid_decomposition 
  RUNTIME DESTINATION  bin/demos/glut/
  )

