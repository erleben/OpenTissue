ADD_EXECUTABLE(finite_element_method src/application.cpp)

TARGET_LINK_LIBRARIES(finite_element_method ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS finite_element_method 
  RUNTIME DESTINATION  bin/demos/glut/
  )
