ADD_EXECUTABLE(unit_closest_points src/unit_closest_points.cpp)

TARGET_LINK_LIBRARIES(unit_closest_points ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_closest_points
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_closest_points unit_closest_points )

