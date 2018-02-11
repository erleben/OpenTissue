ADD_EXECUTABLE(gui_template src/application.cpp)

TARGET_LINK_LIBRARIES(gui_template ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS gui_template
  RUNTIME DESTINATION  bin/demos/glut/
  )


