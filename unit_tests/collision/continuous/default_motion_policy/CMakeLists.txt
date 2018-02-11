ADD_EXECUTABLE(unit_default_motion_policy src/unit_default_motion_policy.cpp)

TARGET_LINK_LIBRARIES(unit_default_motion_policy ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_default_motion_policy
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_default_motion_policy unit_default_motion_policy )

