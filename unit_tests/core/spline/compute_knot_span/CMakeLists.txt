ADD_EXECUTABLE(unit_compute_knot_span src/unit_compute_knot_span.cpp)

TARGET_LINK_LIBRARIES(unit_compute_knot_span ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_compute_knot_span
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_compute_knot_span unit_compute_knot_span )