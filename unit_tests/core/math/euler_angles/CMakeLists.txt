ADD_EXECUTABLE(unit_euler_angles src/unit_euler_angles.cpp)

TARGET_LINK_LIBRARIES(unit_euler_angles ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_euler_angles
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_euler_angles unit_euler_angles )
