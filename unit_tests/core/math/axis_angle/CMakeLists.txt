ADD_EXECUTABLE(unit_axis_angle src/unit_axis_angle.cpp)

TARGET_LINK_LIBRARIES(unit_axis_angle ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_axis_angle
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_axis_angle unit_axis_angle )