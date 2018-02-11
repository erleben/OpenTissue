ADD_EXECUTABLE(unit_clamp src/unit_clamp.cpp)

TARGET_LINK_LIBRARIES(unit_clamp ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_clamp
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_clamp unit_clamp )
