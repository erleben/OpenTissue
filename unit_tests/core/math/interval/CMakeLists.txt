ADD_EXECUTABLE(unit_interval src/unit_interval.cpp)

TARGET_LINK_LIBRARIES(unit_interval ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_interval 
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_interval unit_interval )


