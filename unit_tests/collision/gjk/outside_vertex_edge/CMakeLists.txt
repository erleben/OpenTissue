ADD_EXECUTABLE(unit_outside_vertex_edge src/unit_outside_vertex_edge.cpp)

TARGET_LINK_LIBRARIES(unit_outside_vertex_edge ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_outside_vertex_edge
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_outside_vertex_edge unit_outside_vertex_edge )

