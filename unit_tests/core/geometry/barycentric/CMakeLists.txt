ADD_EXECUTABLE(unit_barycentric src/unit_barycentric.cpp)

TARGET_LINK_LIBRARIES(unit_barycentric ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_barycentric
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_barycentric unit_barycentric  )
