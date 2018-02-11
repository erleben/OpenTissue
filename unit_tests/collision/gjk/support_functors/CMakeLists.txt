ADD_EXECUTABLE(unit_support_functors src/unit_support_functors.cpp)

TARGET_LINK_LIBRARIES(unit_support_functors ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_support_functors
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_support_functors  unit_support_functors )

