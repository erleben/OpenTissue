ADD_EXECUTABLE(unit_timer src/unit_timer.cpp)

TARGET_LINK_LIBRARIES(unit_timer ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_timer
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_timer unit_timer )
