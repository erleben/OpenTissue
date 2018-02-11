ADD_EXECUTABLE(t4_gpu_scan src/application.cpp)

TARGET_LINK_LIBRARIES(t4_gpu_scan ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS t4_gpu_scan
  RUNTIME DESTINATION  bin/demos/glut/
  )


