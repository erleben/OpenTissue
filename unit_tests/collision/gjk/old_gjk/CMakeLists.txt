ADD_EXECUTABLE(unit_obsolete_gjk src/unit_gjk.cpp)

TARGET_LINK_LIBRARIES(unit_obsolete_gjk ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_obsolete_gjk
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_obsolete_gjk unit_obsolete_gjk )

