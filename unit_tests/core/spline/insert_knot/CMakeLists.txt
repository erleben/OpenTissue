ADD_EXECUTABLE(unit_insert_knot src/unit_insert_knot.cpp)

TARGET_LINK_LIBRARIES(unit_insert_knot ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_insert_knot
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_insert_knot unit_insert_knot )