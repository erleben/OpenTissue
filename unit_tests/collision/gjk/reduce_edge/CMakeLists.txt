ADD_EXECUTABLE(unit_reduce_edge src/unit_reduce_edge.cpp)

TARGET_LINK_LIBRARIES(unit_reduce_edge ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_reduce_edge
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_reduce_edge unit_reduce_edge )

