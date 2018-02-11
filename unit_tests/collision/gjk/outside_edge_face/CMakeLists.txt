ADD_EXECUTABLE(unit_outside_edge_face src/unit_outside_edge_face.cpp)

TARGET_LINK_LIBRARIES(unit_outside_edge_face ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_outside_edge_face
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_outside_edge_face unit_outside_edge_face )

