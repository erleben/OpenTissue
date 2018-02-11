ADD_EXECUTABLE(unit_compute_derivatives src/unit_compute_derivatives.cpp)

TARGET_LINK_LIBRARIES(unit_compute_derivatives ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_compute_derivatives
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_compute_derivatives unit_compute_derivatives )