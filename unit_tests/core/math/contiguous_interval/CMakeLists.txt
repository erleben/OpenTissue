ADD_EXECUTABLE(unit_contiguous_interval src/unit_contiguous_interval.cpp)

TARGET_LINK_LIBRARIES(unit_contiguous_interval ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_contiguous_interval
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_contiguous_interval unit_contiguous_interval )