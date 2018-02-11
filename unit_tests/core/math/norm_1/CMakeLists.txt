ADD_EXECUTABLE(unit_norm_1 src/unit_norm_1.cpp)

TARGET_LINK_LIBRARIES(unit_norm_1 ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_norm_1
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_norm_1 unit_norm_1 )