ADD_EXECUTABLE(unit_motion_interpolation src/unit_motion_interpolation.cpp)

TARGET_LINK_LIBRARIES(unit_motion_interpolation ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_motion_interpolation
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_motion_interpolation unit_motion_interpolation )

