ADD_EXECUTABLE(unit_gjk_constants src/unit_constants.cpp)

TARGET_LINK_LIBRARIES(unit_gjk_constants ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_gjk_constants
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_gjk_constants unit_gjk_constants )

