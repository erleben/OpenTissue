ADD_EXECUTABLE(unit_compute_point_on_spline src/unit_compute_point.cpp)

TARGET_LINK_LIBRARIES(unit_compute_point_on_spline ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_compute_point_on_spline
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_compute_point_on_spline unit_compute_point_on_spline )