ADD_EXECUTABLE(unit_conservative_advancement src/unit_conservative_advancement.cpp)

TARGET_LINK_LIBRARIES(unit_conservative_advancement ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_conservative_advancement
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_conservative_advancement unit_conservative_advancement )

