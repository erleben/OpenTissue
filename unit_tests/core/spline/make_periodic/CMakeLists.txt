ADD_EXECUTABLE(unit_make_periodic src/unit_make_periodic.cpp)

TARGET_LINK_LIBRARIES(unit_make_periodic ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_make_periodic
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_make_periodic unit_make_periodic )