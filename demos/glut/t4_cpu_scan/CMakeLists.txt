ADD_EXECUTABLE(t4_cpu_scan src/application.cpp)

TARGET_LINK_LIBRARIES(t4_cpu_scan ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS t4_cpu_scan
  RUNTIME DESTINATION  bin/demos/glut/
  )

