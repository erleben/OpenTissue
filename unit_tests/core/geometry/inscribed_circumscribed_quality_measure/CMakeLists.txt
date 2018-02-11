ADD_EXECUTABLE(unit_inscribed_circumscribed_quality_measure src/unit_inscribed_circumscribed_quality_measure.cpp)
                                                                               
TARGET_LINK_LIBRARIES(unit_inscribed_circumscribed_quality_measure ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_inscribed_circumscribed_quality_measure
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_inscribed_circumscribed_quality_measure unit_inscribed_circumscribed_quality_measure )
