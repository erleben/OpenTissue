ADD_EXECUTABLE(chanvese_segmentation src/application.cpp)

TARGET_LINK_LIBRARIES(chanvese_segmentation ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS chanvese_segmentation
  RUNTIME DESTINATION  bin/demos/glut/
  )
